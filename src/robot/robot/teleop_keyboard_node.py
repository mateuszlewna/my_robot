#!/usr/bin/env python3
import RPi.GPIO as GPIO
import curses
import time

# --- Konfiguracja GPIO ---
# Ważne: Wyłącz ostrzeżenia, bo cleanup() na początku może je wywołać
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) # Ustaw tryb numerowania pinów BCM

# --- Piny silników ---
# PWM dla prędkości
MOTOR1_PWM_PIN = 12
MOTOR2_PWM_PIN = 13

# Piny kierunku (np. dla L298N)
MOTOR1_DIR_A_PIN = 5
MOTOR1_DIR_B_PIN = 6
MOTOR2_DIR_A_PIN = 23
MOTOR2_DIR_B_PIN = 24

# Obiekty PWM (musimy je zainicjować po ustawieniu trybu)
motor1_pwm = None
motor2_pwm = None

speed = 1.0  # Początkowa prędkość (od 0.0 do 1.0 dla PWM)
pwm_frequency = 100 # Częstotliwość PWM w Hz

def setup_gpio():
    # Na początek spróbuj wyczyścić piny, na wypadek gdyby były w użyciu
    try:
        GPIO.cleanup()
    except RuntimeError:
        pass # Ignoruj błąd, jeśli GPIO nie było wcześniej ustawione

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False) # Ponownie wyłącz ostrzeżenia po setmode

    # Ustawienie pinów PWM
    GPIO.setup(MOTOR1_PWM_PIN, GPIO.OUT)
    GPIO.setup(MOTOR2_PWM_PIN, GPIO.OUT)

    # Inicjalizacja obiektów PWM
    global motor1_pwm, motor2_pwm
    motor1_pwm = GPIO.PWM(MOTOR1_PWM_PIN, pwm_frequency)
    motor2_pwm = GPIO.PWM(MOTOR2_PWM_PIN, pwm_frequency)

    # Ustawienie pinów kierunku
    GPIO.setup(MOTOR1_DIR_A_PIN, GPIO.OUT)
    GPIO.setup(MOTOR1_DIR_B_PIN, GPIO.OUT)
    GPIO.setup(MOTOR2_DIR_A_PIN, GPIO.OUT)
    GPIO.setup(MOTOR2_DIR_B_PIN, GPIO.OUT)

    # Ustawienie początkowych wartości
    motor1_pwm.start(0) # Start PWM z 0% wypełnienia
    motor2_pwm.start(0)
    reset_motor_direction()
    print("GPIO zainicjalizowane.")

def set_motor_direction(motor, forward=True):
    if motor == 1:
        GPIO.output(MOTOR1_DIR_A_PIN, GPIO.HIGH if forward else GPIO.LOW)
        GPIO.output(MOTOR1_DIR_B_PIN, GPIO.LOW if forward else GPIO.HIGH)
    elif motor == 2:
        GPIO.output(MOTOR2_DIR_A_PIN, GPIO.HIGH if forward else GPIO.LOW)
        GPIO.output(MOTOR2_DIR_B_PIN, GPIO.LOW if forward else GPIO.HIGH)

def reset_motor_direction():
    GPIO.output(MOTOR1_DIR_A_PIN, GPIO.LOW)
    GPIO.output(MOTOR1_DIR_B_PIN, GPIO.LOW)
    GPIO.output(MOTOR2_DIR_A_PIN, GPIO.LOW)
    GPIO.output(MOTOR2_DIR_B_PIN, GPIO.LOW)

def set_speed(current_speed):
    # PWM przyjmuje wartość procentową od 0 do 100
    duty_cycle = current_speed * 100
    if motor1_pwm:
        motor1_pwm.ChangeDutyCycle(duty_cycle)
    if motor2_pwm:
        motor2_pwm.ChangeDutyCycle(duty_cycle)

def main(stdscr):
    global speed
    curses.cbreak()
    stdscr.nodelay(True)  # nie blokuje na getch()
    stdscr.clear()

    setup_gpio() # Inicjalizacja GPIO tutaj

    set_speed(speed) # Ustaw początkową prędkość

    while True:
        key = stdscr.getch()

        if key == ord('s'): # Jazda do przodu
            set_motor_direction(1, True)
            set_motor_direction(2, True)
            set_speed(speed) # Ustaw prędkość
            stdscr.addstr(0, 0, "Jazda do przodu          ")
        elif key == ord('w'): # Jazda do tyłu
            set_motor_direction(1, False)
            set_motor_direction(2, False)
            set_speed(speed)
            stdscr.addstr(0, 0, "Jazda do tyłu            ")
        elif key == ord('d'): # Skręt w lewo (motor 1 do tyłu, motor 2 do przodu)
            set_motor_direction(1, False)
            set_motor_direction(2, True)
            set_speed(speed)
            stdscr.addstr(0, 0, "Skręt w lewo             ")
        elif key == ord('a'): # Skręt w prawo (motor 1 do przodu, motor 2 do tyłu)
            set_motor_direction(1, True)
            set_motor_direction(2, False)
            set_speed(speed)
            stdscr.addstr(0, 0, "Skręt w prawo            ")
        elif key == curses.KEY_UP: # Zwiększ prędkość
            speed = min(speed + 0.1, 1.0)
            set_speed(speed)
            stdscr.addstr(1, 0, f"Prędkość zwiększona: {speed:.1f}  ")
        elif key == curses.KEY_DOWN: # Zmniejsz prędkość
            speed = max(speed - 0.1, 0.0)
            set_speed(speed)
            stdscr.addstr(1, 0, f"Prędkość zmniejszona: {speed:.1f} ")
        elif key == 27:  # ESC
            break
        else: # Brak kierunku -> zatrzymaj silniki
            reset_motor_direction()
            set_speed(0) # Ustaw prędkość na 0
            stdscr.addstr(0, 0, "Silniki zatrzymane     ")

        stdscr.refresh()
        time.sleep(0.02)

try:
    curses.wrapper(main)

finally:
    # Zatrzymanie PWM przed czyszczeniem
    if motor1_pwm:
        motor1_pwm.stop()
    if motor2_pwm:
        motor2_pwm.stop()
    reset_motor_direction() # Upewnij się, że piny kierunku są zerowe
    GPIO.cleanup() # Ostateczne czyszczenie GPIO
    print("Zakończono program i posprzątano GPIO.")
