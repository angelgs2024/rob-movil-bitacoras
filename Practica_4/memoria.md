
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

En primer lugar, obtenemos el mapa del entorno utilizando `WebGUI.getMap()`.

Posteriormente, esperamos a que el usuario seleccione una posición objetivo mediante `WebGUI.getTargetPose()`.

Antes de aceptar el objetivo comprobamos que:

- Se haya seleccionado una posición válida.
- Sus coordenadas estén dentro de los límites del mapa.
- La celda seleccionada no corresponda a un obstáculo.

También obtenemos la posición inicial del robot mediante `HAL.getPose3d()`.

Como el algoritmo trabaja sobre una matriz de celdas, convertimos ambas posiciones desde coordenadas del mundo a coordenadas del mapa utilizando `WebGUI.worldToGrid()`.

De esta forma, obtenemos dos posiciones:

- `start`: celda inicial del robot.
- `target`: celda objetivo.

### 3.2. Función `get_succesors(pose)`

Esta función obtiene las **ocho celdas vecinas** de una determinada posición del mapa.

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

Esta función permite **penalizar las celdas próximas a obstáculos**.

Para ello, obtenemos los vecinos de la posición mediante `get_succesors()` y comprobamos si alguno corresponde a una celda ocupada.

Si detectamos un obstáculo próximo, devolvemos una penalización de `0.5`. En caso contrario, devolvemos `0`.

```python
if mapa[i[1], i[0]] == 0:
    return 1.5
```

Esta penalización se incorpora al coste de desplazamiento durante la ejecución de Dijkstra.

El objetivo es favorecer las rutas que se encuentren más alejadas de los obstáculos, siempre que el incremento de coste no resulte excesivo.

Esta función únicamente comprueba las ocho celdas vecinas, por lo que no calcula una distancia real a los obstáculos.

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

Después obtenemos sus sucesores y descartamos aquellos que:

- Ya hayan sido explorados.
- Correspondan a obstáculos.
- Supongan un movimiento diagonal que atraviese la esquina entre dos celdas ocupadas.

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

El algoritmo finaliza cuando alcanzamos la posición inicial del robot o cuando no quedan nodos pendientes de explorar.

Como detenemos la expansión al alcanzar `start`, la matriz puede contener celdas que todavía no hayan sido procesadas.

## 5. Reconstrucción y visualización de la ruta

Una vez ejecutado Dijkstra, reconstruimos la ruta utilizando el diccionario `padres`.

Comenzamos desde la posición inicial y vamos obteniendo sucesivamente el siguiente nodo hacia el objetivo:

```python
path = [start]
pose = start

while pose != target:
    pose = padres[pose]
    path.append(pose)
```

La lista `path` contiene una secuencia ordenada de posiciones desde el inicio hasta el destino.

Posteriormente, utilizamos:

```python
WebGUI.showPath(np.array(path))
```

para representar gráficamente la ruta calculada sobre el mapa.

También intentamos mostrar la matriz de distancias mediante:

```python
WebGUI.showNumpy(weights_map)
```

## 6. Navegación y control del robot

### 6.1. Selección del siguiente punto

Después de calcular la ruta, iniciamos el movimiento del robot.

Para ello, utilizamos una segunda cola de prioridad que comienza con la posición inicial.

En cada iteración extraemos el nodo seleccionado:

```python
dist, pose = heapq.heappop(frontier)
frontier = []
```

Vaciamos `frontier` para que en cada paso únicamente se tengan en cuenta los sucesores de la posición actual.

Posteriormente, obtenemos los vecinos y descartamos los obstáculos, los nodos explorados y aquellos cuyo peso no sea inferior al de la celda actual.

Los candidatos se introducen en la cola utilizando:

```python
heapq.heappush(
    frontier,
    (peso + dist_succesor, pose_succesor)
)
```

De esta manera, buscamos avanzar hacia zonas de menor coste, considerando también el coste del siguiente movimiento.

**Es importante diferenciar este procedimiento de la ruta reconstruida:** aunque calculamos y mostramos `path`, el robot no recorre directamente esa lista, sino que selecciona sus siguientes puntos a partir de los pesos de las celdas vecinas.

### 6.2. Cálculo del error de posición

Para desplazarnos hacia cada punto seleccionado, primero convertimos sus coordenadas al sistema del mundo mediante `WebGUI.gridToWorld()`.

Posteriormente, obtenemos la posición actual del robot:

```python
robot = HAL.getPose3d()
```

Y calculamos la distancia euclídea al objetivo

Cuando esta distancia es inferior a `0.8`, consideramos que hemos alcanzado el punto y pasamos a seleccionar el siguiente.

### 6.3. Cálculo del error angular

Calculamos el error angular restando la orientación actual del robot menos la orientación del objetivo.

Normalizamos este error al intervalo $[-\pi,\pi]$:

```python
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

Con este comportamiento buscamos:

- Reducir la velocidad cuando es necesario realizar un giro pronunciado.
- Mantener una velocidad intermedia durante correcciones moderadas.
- Aumentar la velocidad cuando el robot se encuentra correctamente orientado.

A diferencia de un control que establece `v = 0` durante los giros, este método mantiene una velocidad de avance positiva, permitiendo que el robot cambie progresivamente su orientación mientras se desplaza.

### 6.5. Control de velocidad angular

Para controlar la orientación utilizamos un **controlador proporcional**:

Con una ganancia proporcional:

```python
K_p = 1.5
```

En el código:

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

Después de desplazarnos hacia cada punto, actualizamos la posición del robot y calculamos su distancia respecto al objetivo final.

Utilizamos:

```python
distancia = math.hypot(
    target_world[0] - pose_actual.x,
    target_world[1] - pose_actual.y
)
```

Si la distancia es inferior a `1.0`, consideramos que hemos alcanzado el destino.

En ese caso, establecemos las velocidades en cero y salimos del bucle principal:

```python
if distancia < 1.0:
    HAL.setV(0)
    HAL.setW(0)
    break
```

También es posible que la navegación termine si la cola `frontier` queda vacía. En ese caso, el programa deja de seleccionar nuevos puntos, aunque esto no implica necesariamente haber alcanzado el objetivo.

## Video Demostrativo

[Screencast from 2026-11-19 17-20-43.webm](https://github.com/user-attachments/assets/608814e3-f03e-4f0c-b2ab-9fa7e77bc2e0)
