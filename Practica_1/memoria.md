# Basic Vacuum Cleaner

### Objetivo
El objetivo de la práctica es llevar a cabo mediante una máquina de estados un algoritmo para que un robot autónomo sea capaz de recorrer el máximo area posible dentro del mapa dado.

### Componentes
Para esta práctica tenemos disponible un robot con:

* Láser que permite saber a qué distancia se encuentra el obstáculo más cercano
* Motores con los que podemos darle al robot una velocidad lineal y angular

### Métodos auxiliares
Para poder realizar esta práctica he utilizado métodos auxiliares para poder ayudar a simplificar el uso de la información proveniente del laser y que sea más fácil su uso.

* parse_laser_data(laser_data):
  
Función por la cual recibiendo la información del láser retorna dos listas de duplas, una primera dupla con la distancia y el ángulo respecto a los obstáculos (coordenadas polares), y una segunda dupla con las coordenadas cartesianas (x,y).

Código:
```
def parse_laser_data(laser_data):
    """ Parses the LaserData object and returns a tuple with two lists:
        1. List of  polar coordinates, with (distance, angle) tuples,
           where the angle is zero at the front of the robot and increases to the left.
        2. List of cartesian (x, y) coordinates, following the ref. system noted below.

        Note: The list of laser values MUST NOT BE EMPTY.
    """
    laser_polar = []  # Laser data in polar coordinates (dist, angle)
    laser_xy = []  # Laser data in cartesian coordinates (x, y)
    for i in range(180):
        # i contains the index of the laser ray, which starts at the robot's right
        # The laser has a resolution of 1 ray / degree
        #
        #                (i=90)
        #                 ^
        #                 |x
        #             y   |
        # (i=180)    <----R      (i=0)

        # Extract the distance at index i
        dist = laser_data.values[i]
        # The final angle is centered (zeroed) at the front of the robot.
        angle = math.radians(i - 90)
        laser_polar += [(dist, angle)]
        # Compute x, y coordinates from distance and angle
        x = dist * math.cos(angle)
        y = dist * math.sin(angle)
        laser_xy += [(x, y)]
    return laser_polar, laser_xy
```

* get_min(laser_polar)

Función que retorna el valor mínimo de las listas de tuplas que se le pasa como argumento, que hace referencia a las mediciones del laser. Este valor mínimo viene a significar el obstaculo más cercano al robot.

Código:
```
def get_min(laser_polar):
    """ Iterate through a list to get the minimum value
    We are going to compare all the values of the list to get the minimum
    """
    minimum = laser_polar[0]

    for i in laser_polar:
        if i[0] < minimum[0]:
            minimum = i

    return minimum
```

### Funcionamiento

El código de esta práctica se basa en una máquina de estados finitos que representa el funcionamiento de la navegación local de nuestro robot.
Para esta máquina de estados hemos creado 4 estados: FORWARD, BACK, TURN, SPIN.

* FORWARD:

  El estado FORWARD se activa al principio de la ejecución y en este estado lo que se da es una velocidad angular (w) = 0.125 y una velocidad linear (v) = 0.25. Esta elección de valores se debe a que con esta velocidad linear si buscamos cambiar de estados cuando detectemos un obstáculo a menos de 0.25 m hace que no nos choquemos mientras se lleva a cabo la transición. Y el añadirle un valor a w en vez de que el movimiento sea recto es para que tenga mayor facilidad para entrar a en habitaciones y pasillos, ya que con un movimiento linear se tendría que alinear perfectamente la trayectoria del robot con la entrada del pasillo o puerta.

  La condición que se tiene que cumplir para cambiar de estados es que detectemos un obstáculo a menos de 0.25 m, hemos elegido esta distancia ya que es la distancia máxima por la que podemos entrar por la puerta de la habitación de la izquierda sin que la detecte como un obstáculo.

  Si detectamos el obstáculo transicionamos al estado BACK sino continuamos en este estado.
  Además en cada iteración que estemos dentro de ese estado se pide elegir un numero decimal aleatorio entre 0 y 1 y si ese número es menor que 0.001 transicionamos al estado SPIN, elegimos este numero para que no transicione rápido sino que le de tiempo a moverse al robot.
  
* BACK:

  El estado BACK se activa cuando se encuentra un obstaculo a menos de 0.25 m del robot.

  Lo que hacemos en este estado es darnos la vuelta (360 grados) fijando la velocidad linear a 0 y la velocidad angular a 3.14 rad y cuando el obstáculo más cercano está a mas de 0.25 m transicionamos al estado TURN.
  
* TURN:

  El estado TURN se activa justo después de ejecutarse el estado BACK y en él lo que hacemos es girar un ángulo aleatorio entre 90 grados y -90 grados para poder aportar al sistema mayor aleatoriedad.

  Después de ejecutarse lo que hacemos es transicionar al estado FORWARD ya que sabemos con seguridad que como el movimiento va a ser en sentido contrario al obstaculo vamos a estar seguros.
  
* SPIN:

  El estado SPIN como he comentado anteriormente se ejecuta cuando el número aleatorio generado en FORWARD es menor que 0.001

  En el estado SPIN lo que hacemos es generar un movimiento en espiral para optimizar la limpieza ya que con este movimiento barremos un gran porcentaje de espacio.

  Para generar ese movimiento en espiral establecemos en un principio los valores de r = 0 y w = 0.5, ya que v = w * r, y para ir haciendo que el tamaño de la espiral vaya aumentando lo que hacemos es por cada iteración ir aumentando el valor de r 0.0001 unidades hasta llegar a un valor máximo de 0.25 que es el valor en el que podemos asegurar que el robot no se choca con la configuración actual. Si llegamos a este valor reiniciamos su valor a 0.

  Si detecta un  un obstáculo a menos de 0.25 m transiciona al estado BACK.

    
