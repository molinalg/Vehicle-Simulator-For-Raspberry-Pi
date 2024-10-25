import time, sys, signal, json, threading, os
import RPi.GPIO as GPIO

# Pines de sensores y actuadores del vehículo
MOTOR_A_PIN, MOTOR_B_PIN, MOTOR_E_PIN = 5, 6, 13    # Motor
SERVO_PIN = 16                                      # Servomotor
BUTTON_PIN = 23                                     # Botón
LIGHT_PIN = 4                                       # LDR
SOUND_TRIGGER_PIN, SOUND_ECHO_PIN = 24, 25          # Ultrasonidos

vehicleControlCommands = None                                       # Comandos extraidos del JSON
current_steering, current_speed, current_duration = 0, 0, 0.0       # Valores del comando actual del servomotor
current_light = 0                                                   # Valor de la última medición de luz tomada
current_led_intensity = 0                                           # Valor de las leds cuando no están los intermitentes

slowing_down = None     # Variable para controlar las luces de frenado (0 = deben mantenerse apagadas,
                        # 1 = deben activarse, 2 = deben mantenerse encendidas, 3 = deben desactivarse)

lights_on = None                                                    # Valor que indica si las luces de posicion están encendidas
previous_blinker = False                                            # Valor que indica si el comando anterior encendía intermitentes

motor_dc_object = None                                              # Motor
servo_object = None                                                 # Servomotor

t1, t2, t3 = None, None, None                                       # Hilos para sensores y actuadores

R_RED_PIN, R_GREEN_PIN, R_BLUE_PIN = 14, 15, 18     # Pines RGB para LED derecho
L_RED_PIN, L_GREEN_PIN, L_BLUE_PIN = 22, 17, 27     # Pines RGB para LED izquierdo

led_right_r, led_right_g, led_right_b = None, None, None            # Objetos de control de colores en LED derecho
led_left_r, led_left_g, led_left_b = None, None, None               # Objetos de control de colores en LED izquierdo

# Manejo del estado del botón y del sistema
should_run, executed_command = False, False                         # Estados del botón para indicar ejecución o no
keep_reading = False                                                # Variable para controlar parones ajenos al botón
sys_state_lock = threading.Lock()                                   # Garantiza el cambio de estado del sistema


def load_commands() -> None:
    """Carga los comandos del servomotor y motor"""
    global vehicleControlCommands

    # Se saca la ruta actual del script
    current_directory = os.path.dirname(os.path.realpath(__file__))

    # Se junta la ruta con el json para obtener el path completo
    file_json = 'commands.json'
    path_json = os.path.join(current_directory, file_json)

    # Se cargan los comandos del json
    with open(path_json, "r") as file:
        vehicleControlCommands = json.load(file)


def set_angle_to_percent(angle) -> float:
    """Establece ángulo de giro en el servomotor"""
    start, end = 4, 12.5

    angle = max(0, min(180, angle))
    ratio = (end-start)/180     # Compute ratio from angle to percent
    angle_as_percent = angle * ratio

    return start + angle_as_percent


def execute_command(command) -> None:
    global current_steering, current_speed, current_led_intensity
    global slowing_down
    global lights_on
    global previous_blinker

    # Primero se mira si se está frenando
    if current_speed > command["Speed"]:
        # Si lo está se mira si va a activar intermitentes para reiniciar variables en ese caso
        if command["SteeringAngle"] >= 80 and command["SteeringAngle"] <= 100:
            # Se pide que se activen las luces de frenado
            if slowing_down == 0:
                slowing_down = 1
                previous_blinker = False
        else:
            # Se reinician variables
            lights_on = False
            slowing_down = 0
            current_led_intensity = 0
            previous_blinker = True

    else:
        if command["SteeringAngle"] >= 80 and command["SteeringAngle"] <= 100:
            # Se pide que se desactiven las luces de frenado si estaban activadas
            if not previous_blinker:
                if slowing_down == 2:
                    slowing_down = 3
        else:
            # Se reinician variables
            lights_on = False
            slowing_down = 0
            current_led_intensity = 0
            previous_blinker = True

    current_steering, current_speed = command["SteeringAngle"], command["Speed"]

    print("===========================================================")
    print("Moviendo el servomotor al ángulo {}".format(current_steering))
    move_servo_motor()
    print("Servomotor movido")

    print("Moviendo el motor de CC con velocidad {}".format(current_speed))
    move_dc_motor()
    print("Motor movido")
    print("===========================================================")

    vehicle_timer(command["Time"])


def vehicle_timer(command_time) -> None:
    """Mide el tiempo que lleva ejecutado el comando actual del vehículo.
       El tiempo se reinicia si se completa el comando"""
    global current_duration
    global vehicleControlCommands

    #print("Tiempo que el comando debe ejecutarse: {}".format(command_time))
    while current_duration < command_time:
        if not should_run:
            return
        current_duration += 0.1
        time.sleep(0.1)
        #print("Tiempo de ejecución del comando: {} segundos de {} segundos".format(current_duration, command_time))

    current_duration = 0


def vehicle_controller() -> None:
    """Función para controlar la velocidad del motor"""
    global vehicleControlCommands

    # Ejecución de los comandos mientras haya o el vehículo esté activo
    while should_run and vehicleControlCommands:
        execute_command(vehicleControlCommands[0])
        # Detención del vehículo por solicitud
        if not should_run:
            servo_object.ChangeDutyCycle(set_angle_to_percent(90))
            motor_dc_object.ChangeDutyCycle(0.0)
            break
        del vehicleControlCommands[0]

    # Detención del vehículo por falta de comandos
    if not vehicleControlCommands:
        print("Parando el vehículo...")
        execute_command({"SteeringAngle": 90.0, "Speed": 0.0, "Time": 1.0})
        print("Vehículo parado")


def signal_handler(sig, frame) -> None:
    """Función que limpia los puertos y termina el proceso"""
    print("Saliendo")
    servo_object.stop()
    motor_dc_object.stop()
    GPIO.cleanup()
    sys.exit(0)


def setup_devices():
    """Función para configurar dispositivos"""
    global servo_object, motor_dc_object
    global led_right_r, led_right_g, led_right_b
    global led_left_r, led_left_g, led_left_b
    global slowing_down

    GPIO.setmode(GPIO.BCM)

    # Configuración botón
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Configuración ultrasonidos
    GPIO.setup(SOUND_ECHO_PIN, GPIO.IN)
    GPIO.setup(SOUND_TRIGGER_PIN, GPIO.OUT)

    # Configuración servomotor
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    servo_object = GPIO.PWM(SERVO_PIN, 50)
    servo_object.start(0)
    servo_object.start(set_angle_to_percent(90))

    # Configuración motor DC
    GPIO.setup(MOTOR_A_PIN, GPIO.OUT)
    GPIO.setup(MOTOR_B_PIN, GPIO.OUT)
    GPIO.setup(MOTOR_E_PIN, GPIO.OUT)

    motor_dc_object = GPIO.PWM(MOTOR_E_PIN, 100)

    GPIO.output(MOTOR_A_PIN, True)
    GPIO.output(MOTOR_B_PIN, False)
    GPIO.output(MOTOR_E_PIN, True)

    motor_dc_object.start(0)

    # Configuración de los LEDs
    GPIO.setup(R_RED_PIN, GPIO.OUT)
    GPIO.setup(R_GREEN_PIN, GPIO.OUT)
    GPIO.setup(R_BLUE_PIN, GPIO.OUT)

    GPIO.setup(L_RED_PIN, GPIO.OUT)
    GPIO.setup(L_GREEN_PIN, GPIO.OUT)
    GPIO.setup(L_BLUE_PIN, GPIO.OUT)

    led_right_r = GPIO.PWM(R_RED_PIN,100)
    led_right_g = GPIO.PWM(R_GREEN_PIN,100)
    led_right_b = GPIO.PWM(R_BLUE_PIN,100)
    led_left_r = GPIO.PWM(L_RED_PIN,100)
    led_left_g = GPIO.PWM(L_GREEN_PIN,100)
    led_left_b = GPIO.PWM(L_BLUE_PIN,100)

    led_right_r.start(0)
    led_right_g.start(0)
    led_right_b.start(0)
    led_left_r.start(0)
    led_left_g.start(0)
    led_left_b.start(0)

    # Se inicializa la variable slowing_down a 0
    slowing_down = 0

    print("La configuración de los dispositivos ha finalizado")


def trigger_led_pin(pin, intensity):
    """Función para modificar la intensidad actual un pin de una luz LED"""
    global current_led_intensity
    pin.ChangeDutyCycle(current_led_intensity + intensity)


def led_pin_off(pin):
    """Función para apagar un pin de una luz LED"""
    pin.ChangeDutyCycle(0)


def led_red(red_pin, green_pin, blue_pin, intensity):
    """Función para poner una luz LED en rojo"""
    trigger_led_pin(red_pin, intensity)
    led_pin_off(green_pin)
    led_pin_off(blue_pin)


def led_white(red_pin, green_pin, blue_pin, intensity):
    """Función para poner una luz LED en blanco"""
    trigger_led_pin(red_pin, intensity)
    trigger_led_pin(green_pin, intensity)
    trigger_led_pin(blue_pin, intensity)


def led_yellow(red_pin, green_pin, blue_pin, intensity):
    """Función para poner una luz LED en amarillo"""
    trigger_led_pin(red_pin, intensity)
    trigger_led_pin(green_pin, intensity)
    led_pin_off(blue_pin)


def led_turn_off(red_pin, green_pin, blue_pin):
    """Función para apagar una luz LED"""
    led_pin_off(red_pin)
    led_pin_off(green_pin)
    led_pin_off(blue_pin)


def start_vehicle() -> None:
    """Función para arrancar el vehículo"""
    global t1, t2, t3
    global executed_command

    # Damos por completado el comando
    sys_state_lock.acquire()
    executed_command = True
    sys_state_lock.release()

    # Hilo que gestiona el sistema del LDR
    t1 = threading.Thread(target=ldr_manager, daemon=True)
    t1.start()
    # Hilo que gestiona el sistema del Ultrasonidos
    t2 = threading.Thread(target=ultrasonic_manager, daemon=True)
    t2.start()
    # Hilo que gestiona el motor y servomotor
    t3 = threading.Thread(target=vehicle_controller, daemon=True)
    t3.start()
    # Hilo que gestiona las luces LED
    t4 = threading.Thread(target=led_manager, daemon=True)
    t4.start()

    t1.join()
    t2.join()
    t3.join()
    t4.join()


def move_dc_motor() -> None:
    """Función para cambiar la velocidad del motor"""
    global motor_dc_object
    motor_dc_object.ChangeDutyCycle(current_speed)


def move_servo_motor() -> None:
    """Función para cambiar el ángulo del servomotor"""
    global servo_object
    servo_object.ChangeDutyCycle(set_angle_to_percent(current_steering))


def ultrasonic_manager() -> None:
    """Función para devolver la distancia del sensor"""
    global should_run
    global keep_reading
    global executed_command

    while should_run or keep_reading:
        GPIO.output(SOUND_TRIGGER_PIN, True)
        # Se crea un tiempo para el pulso del trigger de 0.01 milisegundos
        time.sleep(0.00001)
        GPIO.output(SOUND_TRIGGER_PIN, False)

        # Registrar tiempo de activación
        while 0 == GPIO.input(SOUND_ECHO_PIN):
            start_time = time.time()

        # Registrar tiempo de desactivación
        while 1 == GPIO.input(SOUND_ECHO_PIN):
            stop_time = time.time()

        # Cálculo de la duración de la activación
        time_elapsed = stop_time - start_time
        # La distancia se calcula en base a la velocidad del sonido (342 m/s = 34300 cm/s)
        distance = (time_elapsed * 34300) / 2  # Se divide entre 2 porque solo interesa la ida/vuelta

        # Imprimimos la distancia en consola
        print("[D] La distancia es: {} cm".format(distance))
        # Si la distancia es menos de 20 cm se para el circuito pero se sigue manteniendo esta función en funcionamiento
        if distance < 20:
            should_run = False
            keep_reading = True
        else:
            if keep_reading:
                sys_state_lock.acquire()
                should_run = True
                keep_reading = False
                executed_command = False
                sys_state_lock.release()
                # Se acaba con la función para que pueda acabar el hilo y lanzarse de nuevo
                break

        # Para no saturarnos de mediciones establecemos que se tome una medición cada medio segundo
        time.sleep(2)


def ldr_manager() -> None:
    """Función para comprobar el tiempo de carga del capacitor"""
    global current_light
    while should_run:
        # Inicializamos la variable de recuento
        count = 0

        # Configuramos el pin de luminosidad como salida y en bajo
        GPIO.setup(LIGHT_PIN, GPIO.OUT)
        GPIO.output(LIGHT_PIN, GPIO.LOW)

        # Esperamos 10ms
        time.sleep(0.01)  # REVISAR ESTO -> PONE EN EL ENUNCIADO 10ms PERO EN LA FOTO PONE 0.1 QUE SERÍAN 100

        # Configuramos pin de luminosidad como entrada
        GPIO.setup(LIGHT_PIN, GPIO.IN)

        # Bucle para esperar a que se ponga alto el pin
        # Se va contando en la variable de recuento para ver cuanto tarda
        while GPIO.input(LIGHT_PIN) == GPIO.LOW:
            count += 1

        #print("Valor obtenido del capacitor: {}".format(count))
        current_light = count
        time.sleep(2)


def led_manager() -> None:
    global slowing_down
    global lights_on
    global current_led_intensity
    lights_on = False

    # Tiempo que pasa el intermitente en cada estado del parpadeo
    freq_intermitente = 1/6

    while should_run:
        # Primero comprobamos si se debe encender un intermitente
        if current_steering > 100:
            # Encendemos intermitente izquierdo
            print("----------Enciendo intermitente izquierdo----------")
            while current_steering > 100 and should_run:
                # Se enciende y se apaga 3 veces por segundo
                led_turn_off(led_right_r, led_right_g, led_right_b)
                led_turn_off(led_left_r, led_left_g, led_left_b)
                current_led_intensity = 0
                for i in range(3):
                    led_yellow(led_left_r, led_left_g, led_left_b, 100)
                    time.sleep(freq_intermitente)
                    led_turn_off(led_left_r, led_left_g, led_left_b)
                    time.sleep(freq_intermitente)
            print("----------Apago intermitente izquierdo----------")

        elif current_steering < 80:
            # Encendemos el intermitente derecho
            print("----------Enciendo intermitente derecho----------")
            while current_steering < 80 and should_run:
                # Se enciende y se apaga 3 veces por segundo
                led_turn_off(led_right_r, led_right_g, led_right_b)
                led_turn_off(led_left_r, led_left_g, led_left_b)
                current_led_intensity = 0
                for i in range(3):
                    led_yellow(led_right_r, led_right_g, led_right_b, 100)
                    time.sleep(freq_intermitente)
                    led_turn_off(led_right_r, led_right_g, led_right_b)
                    time.sleep(freq_intermitente)
            print("----------Apago intermitente derecho----------")

        else:
            # Si la iluminación es baja y las luces de posición están apagadas, se aumenta intensidad al 50 en rojo
            if current_light > 2000 and not lights_on:
                print("----------Enciendo luces de posición----------")
                led_red(led_right_r, led_right_g, led_right_b, 50)
                led_red(led_left_r, led_left_g, led_left_b, 50)
                current_led_intensity = current_led_intensity + 50
                lights_on = True

            # Si la iluminación es alta y las luces de posición están encendidas, se disminuye intensidad al 50 en rojo
            elif current_light <= 2000 and lights_on:
                print("----------Apago luces de posición----------")
                led_red(led_right_r, led_right_g, led_right_b, -50)
                led_red(led_left_r, led_left_g, led_left_b, -50)
                current_led_intensity = current_led_intensity - 50
                lights_on = False

            if slowing_down == 1:
                # Si está frenando se aumenta un 50 la actual luz roja trasera
                print("----------Enciendo luz de frenado----------")
                led_red(led_right_r, led_right_g, led_right_b, 50)
                led_red(led_left_r, led_left_g, led_left_b, 50)
                current_led_intensity = current_led_intensity + 50
                slowing_down = 2

            elif slowing_down == 3:
                # Si no está frenando y antes lo estaba se disminuye un 50 la actual luz roja trasera
                print("----------Apago luz de frenado----------")
                led_red(led_right_r, led_right_g, led_right_b, -50)
                led_red(led_left_r, led_left_g, led_left_b, -50)
                current_led_intensity = current_led_intensity - 50
                slowing_down = 0

        time.sleep(0.5)


def button_manager() -> None:
    """Función para el control del botón"""
    global should_run, executed_command

    while True:
        if not GPIO.input(BUTTON_PIN):
            while not GPIO.input(BUTTON_PIN):
                pass
            sys_state_lock.acquire()
            should_run = not should_run
            executed_command = False
            sys_state_lock.release()
        time.sleep(0.1)


if __name__ == '__main__':
    try:
        load_commands()
        setup_devices()

        # Hilo para controlar el manejo del botón
        t0 = threading.Thread(target=button_manager, daemon=True)
        t0.start()

        while True:
            # Se enciende el vehículo cuando should run es True y executed command es False
            if should_run and not executed_command:
                start_vehicle()
                time.sleep(0.2)

    except Exception as e:
        signal.signal(signal.SIGINT, signal_handler)
        signal.pause()
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal_handler)
        signal.pause()