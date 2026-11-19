
# Global Navigation

## 1. Objetivo

El objetivo de esta práctica es implementar un sistema de **navegación global para un robot móvil**, capaz de calcular una ruta desde su posición inicial hasta un destino seleccionado en un mapa, evitando los obstáculos presentes en el entorno.

Para ello, utilizamos el **algoritmo de Dijkstra**, que permite calcular los costes de desplazamiento entre las distintas celdas del mapa.

El sistema se divide en dos partes principales:

- **Planificación global:** calcular los costes de las celdas y obtener una ruta desde la posición inicial hasta el objetivo.
- **Control del movimiento:** seleccionar sucesivamente los puntos de menor coste y controlar las velocidades lineal y angular del robot para desplazarse hacia ellos.

Además, introducimos una penalización para las celdas próximas a obstáculos, con el objetivo de favorecer rutas que mantengan una mayor separación respecto a ellos.

## 2. Componentes utilizados

Para desarrollar la práctica utilizamos las siguientes librerías:

| Librería | Función |
|---|---|
| `WebGUI` | Obtener el mapa y el objetivo, convertir coordenadas y visualizar la ruta y el mapa de distancias. |
| `HAL` | Obtener la posición y orientación del robot y establecer las velocidades de movimiento. |
| `Frequency` | Regular la frecuencia del bucle de control. |
| `heapq` | Implementar una cola de prioridad para seleccionar los nodos de menor coste. |
| `NumPy` | Almacenar y manipular la matriz de distancias. |
| `math` | Realizar cálculos de distancias, ángulos y funciones trigonométricas. |

## 3. Métodos auxiliares

### 3.1. Obtención de las posiciones inicial y objetivo

Las celdas con valor `0` se consideran obstáculos.

Al comenzar cada ciclo, leemos la pose del robot mediante `HAL.getPose3d()`. Convertimos sus coordenadas del mundo con `WebGUI.worldToGrid()` y las pasamos a enteros para obtener la celda inicial `start`.

A continuación, esperamos un objetivo con `WebGUI.getTargetPose()`. Si obtenemos una posición, también la convertimos a una celda entera `target`. Aceptamos el objetivo cuando la celda está dentro del mapa, está libre y difiere de `ultimo_objetivo`. Ambas variables se comparan en coordenadas de la cuadrícula. `ultimo_objetivo` se inicializa a `None` y se actualiza cuando el robot llega al destino.

### 3.2. Función `get_succesors(pose)`

Esta función obtiene hasta ocho celdas vecinas de una determinada posición del mapa.

Para cada sucesor comprobamos que sus coordenadas se encuentren dentro de los límites de la matriz.

Utilizamos un sistema de navegación de ocho conexiones, que permite realizar movimientos horizontales, verticales y diagonales.

Los costes de desplazamiento son:

| Movimiento | Coste |
|---|---|
| Horizontal | 1 |
| Vertical | 1 |
| Diagonal | √2 ≈ 1,414 |

Finalmente, la función devuelve un diccionario donde cada clave representa una posición vecina y su valor corresponde al coste de desplazamiento.

### 3.3. Función `near_obstacles(pose)`

Esta función permite penalizar las celdas próximas a obstáculos.

Para ello, obtenemos los vecinos de la posición mediante `get_succesors()` y comprobamos si alguno corresponde a una celda ocupada.

Si detectamos un obstáculo próximo, devolvemos una penalización de `1.5`. En caso contrario, devolvemos `0`.

```python
if mapa[i[1], i[0]] == 0:
    return 1.5
```

Esta penalización se incorpora al coste de desplazamiento durante la ejecución de Dijkstra.

La penalización se aplica una sola vez por celda si al menos uno de sus vecinos es un obstáculo. La comprobación se limita al vecino inmediato.

## 4. Planificación global mediante Dijkstra

### 4.1. Inicialización

Implementamos Dijkstra comenzando desde la posición objetivo y expandiéndonos hacia el resto del mapa.

Para ello, utilizamos las siguientes estructuras:

- `frontier`: cola de prioridad con los nodos pendientes de explorar.
- `explored`: lista de nodos que ya han sido procesados.
- `distancias`: matriz que almacena el menor coste encontrado para llegar al objetivo.
- `padres`: diccionario que permite reconstruir posteriormente la ruta.

Inicializamos la matriz de distancias con valores infinitos:

```python
distancias = np.full(mapa.shape, np.inf)
distancias[target[1], target[0]] = 0
```

Asignamos una distancia de cero al objetivo, ya que es el punto desde el que comienza la expansión.

### 4.2. Expansión de los nodos

Mientras existan elementos en `frontier`, extraemos el nodo con menor coste acumulado mediante:

```python
dist, pose = heapq.heappop(frontier)
```

Una vez seleccionado, comprobamos que no haya sido explorado anteriormente.

Después obtenemos sus sucesores y descartamos las celdas exploradas y las ocupadas.

### 4.3. Cálculo de las distancias

Para cada sucesor válido calculamos su nuevo coste:

```python
new_dist = dist + dist_succesor + near_obstacles(pose_succesor)
```

Si el nuevo coste es inferior al almacenado anteriormente, actualizamos la matriz de distancias y añadimos el sucesor a la cola de prioridad.

Además, guardamos su nodo padre:

```python
padres[pose_succesor] = pose
```

De esta forma, cada nodo almacena el siguiente punto hacia el objetivo correspondiente al mejor coste encontrado.

El algoritmo finaliza cuando alcanzamos la posición inicial del robot o cuando no quedan nodos pendientes de explorar en este caso devolvemos None.

## 5. Reconstrucción y visualización de la ruta

Tras calcular los costes, mostramos la matriz con `WebGUI.showNumpy(weights_map)` y reconstruimos la ruta utilizando el diccionario `padres`.

Comenzamos desde la posición inicial y vamos obteniendo sucesivamente el siguiente nodo hacia el objetivo:

```python
path = [start]
pose = start

while pose != target:
    pose = padres[pose]
    path.append(pose)
```

La lista `path` contiene una secuencia ordenada de posiciones desde el inicio hasta el destino.

Mostramos la ruta en el mapa e imprimimos el número de desplazamientos entre celdas. Como `path` incluye la celda inicial, una ruta con 18 celdas tiene 17 pasos.

## 6. Navegación y control del robot

### 6.1. Selección del siguiente punto

Utilizamos una segunda cola de prioridad que comienza con la posición inicial.

En cada iteración extraemos el nodo seleccionado:

```python
dist, pose = heapq.heappop(frontier)
frontier = []
```

Vaciamos `frontier` para que en cada paso únicamente se tengan en cuenta los sucesores de la posición actual.

Posteriormente, obtenemos los vecinos. Descartamos los obstáculos, los nodos explorados y las celdas con peso infinito o con un peso mayor o igual que el de la celda actual.

Los candidatos se introducen en la cola utilizando:

```python
heapq.heappush(
    frontier,
    (peso + dist_succesor, pose_succesor)
)
```

De esta manera, buscamos avanzar hacia zonas de menor coste, considerando también el coste del siguiente movimiento.

En cada paso se elige el vecino con menor coste

### 6.2. Cálculo de la distancia

Para desplazarnos hacia cada punto seleccionado, primero convertimos sus coordenadas al sistema del mundo mediante `WebGUI.gridToWorld()`.

Posteriormente, obtenemos la posición actual del robot:

```python
robot = HAL.getPose3d()
```

Calculamos la distancia euclídea al punto seleccionado mediante `math.hypot(dx, dy)`

### 6.3. Cálculo del error angular

Calculamos el error angular restando la orientación actual del robot (`robot.yaw`) al rumbo deseado hacia el punto (`math.atan2(dy, dx)`). A continuación, normalizamos el error al intervalo $[-\pi,\pi]$.

```python
error = math.atan2(dy, dx) - robot.yaw
error = math.atan2(math.sin(error), math.cos(error))
```

Esta normalización permite obtener la diferencia angular más corta, evitando problemas cuando los ángulos atraviesan los límites de $-\pi$ y $\pi$.

### 6.4. Control de velocidad lineal

La velocidad lineal se establece en función del error angular y de la distancia al objetivo.

En el código:

```python
if abs(error) > 1.0:
    v = 0.4
elif abs(error) > 0.4:
    v = 0.8
else:
    v = min(1.5, 1.5 * distancia)
```

La velocidad de avance es menor cuando el robot debe corregir un error angular grande. Cuando el error angular es de hasta `0.4` radianes, ajustamos la velocidad en función de la distancia al punto, con un máximo de `1.5`.

### 6.5. Control de velocidad angular

Para controlar la orientación utilizamos un **controlador proporcional** con una ganancia de `1.5` y una saturación entre `-0.8` y `0.8`.

```python
w = max(-0.8, min(0.8, 1.5 * error))
```

De esta manera, la velocidad angular aumenta proporcionalmente al error, pero nunca supera los límites establecidos.

Finalmente, enviamos las velocidades al robot:

```python
HAL.setV(v)
HAL.setW(w)
```

El bucle de control utiliza `Frequency.tick()` para regular la frecuencia con la que se actualizan estas órdenes.

## 7. Condición de finalización

Después de procesar cada punto, volvemos a leer la posición del robot, calculamos la distancia al objetivo final e imprimimos ese valor en la consola.

```python
distancia = math.hypot(
    target_world[0] - pose_actual.x,
    target_world[1] - pose_actual.y
)
print(distancia)
```

Si la distancia es inferior a `1.0`, consideramos que hemos alcanzado el destino.

En ese caso, ponemos ambas velocidades a cero, guardamos la celda alcanzada y salimos del bucle `while frontier`.

```python
if distancia < 1.0:
    HAL.setV(0)
    HAL.setW(0)
    ultimo_objetivo = target
    frontier = None
    break
```

## Video Demostrativo

[Screencast from 2026-11-19 17-54-56.webm](https://github.com/user-attachments/assets/e3cdb38c-224a-48ff-a693-fa37786f7f80)

