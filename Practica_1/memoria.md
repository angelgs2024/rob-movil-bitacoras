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
    
