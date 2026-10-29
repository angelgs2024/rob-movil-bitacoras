
# Navegación mediante fuerzas virtuales

## Objetivo

El objetivo de esta práctica es implementar un algoritmo de **navegación autónoma mediante fuerzas virtuales**, que permita al robot desplazarse hacia una serie de objetivos mientras evita los obstáculos presentes en el entorno.

Para conseguirlo, utilizo la información proporcionada por un **sensor láser**, que permite detectar la distancia y dirección de los obstáculos, y la posición relativa del objetivo respecto al robot.

El sistema combina dos componentes principales:

- **Fuerza de atracción:** dirige al robot hacia el objetivo que debe alcanzar.
- **Fuerza de repulsión:** modifica la dirección de navegación para evitar los obstáculos detectados.

A partir de la combinación de ambas fuerzas, calculo una dirección resultante que utilizo para controlar la **velocidad angular mediante un controlador proporcional (P)**, manteniendo una velocidad lineal constante.

## Componentes

Para esta práctica tenemos disponible un robot con:

- **Sensor láser:** proporciona 180 medidas de distancia, con una resolución angular de 1 grado, cubriendo aproximadamente los 180 grados delanteros del robot.
- **Sistema de posicionamiento:** permite obtener la posición `(x, y)` y la orientación `yaw` del robot.
- **Sistema de movimiento:** permite controlar la velocidad lineal mediante `HAL.setV()` y la velocidad angular mediante `HAL.setW()`.
- **Interfaz de visualización:** mediante `WebGUI` podemos consultar los objetivos de navegación y representar las fuerzas calculadas.

La implementación se realiza en **Python**, utilizando principalmente la biblioteca `math` para los cálculos trigonométricos y vectoriales.

## Métodos auxiliares

### 1. Procesamiento de datos del láser: `parse_laser_data()`

Para interpretar las medidas del sensor láser, utilizo una función que recibe los datos de distancia y los transforma en coordenadas polares y cartesianas.

El sensor proporciona **180 rayos**, distribuidos desde el lateral derecho hasta el izquierdo del robot.

Para cada medida calculo su ángulo respecto al eje frontal:

```python
angle = math.radians(i - 90)
```

De esta manera:

- `i = 0`: ángulo de -90°, correspondiente al lateral derecho.
- `i = 90`: ángulo de 0°, correspondiente al frente.
- `i = 179`: ángulo de +89°, correspondiente al lateral izquierdo.

Posteriormente, transformo las medidas a coordenadas cartesianas:

```python
x = dist * math.cos(angle)
y = dist * math.sin(angle)
```

Así obtengo dos listas:

- **`laser_polar`:** contiene las parejas `(distancia, ángulo)`.
- **`laser_xy`:** contiene las coordenadas `(x, y)` de cada medida.

Aunque calculo ambas representaciones, en el algoritmo de evasión utilizo únicamente las coordenadas polares.

### 2. Transformación de coordenadas: `absolute2relative()`

Los objetivos de navegación están definidos mediante coordenadas absolutas del entorno. Sin embargo, para calcular la dirección que debe seguir el robot necesito expresar su posición en el **sistema de referencia local del robot**.

Primero calculo la diferencia entre la posición del objetivo y la posición del robot:

```python
dx = x_abs - robotx
dy = y_abs - roboty
```

Posteriormente, realizo una rotación utilizando el ángulo de orientación del robot:

```python
x_rel = dx * math.cos(-robott) - dy * math.sin(-robott)
y_rel = dx * math.sin(-robott) + dy * math.cos(-robott)
```

Con esta transformación consigo que el objetivo quede expresado respecto a la posición y orientación actuales del robot.

Esto permite determinar si el objetivo se encuentra delante, detrás, a la derecha o a la izquierda, independientemente de la orientación global del robot.

### 3. Obtención del objetivo: `pos_target()`

Para obtener el siguiente punto de navegación utilizo:

```python
currentTarget = WebGUI.getNextTarget()
```

A continuación, extraigo sus coordenadas absolutas y las transformo al sistema de referencia local mediante la función `absolute2relative()`.

```python
target_x, target_y = absolute2relative(
    currentTarget_x,
    currentTarget_y,
    robot_pos.x,
    robot_pos.y,
    robot_pos.yaw
)
```

La función devuelve:

```python
return target_x, target_y
```

Estas coordenadas se utilizan posteriormente para calcular el vector de atracción hacia el objetivo.

Para la visualización, en cambio, utilizo las coordenadas absolutas del objetivo:

```python
target = [
    currentTarget.getPose().x,
    currentTarget.getPose().y
]

WebGUI.showLocalTarget(target)
```

De esta manera, diferencio las coordenadas utilizadas para el control de las enviadas a la interfaz de visualización.

La representación correcta del punto dependerá de si `showLocalTarget()` interpreta dichas coordenadas como absolutas o relativas.

### 4. Cálculo de la fuerza de repulsión: `get_laser_vector()`

Esta función constituye una de las partes principales del algoritmo, ya que calcula la influencia de los obstáculos sobre la trayectoria del robot.

He decidido trabajar únicamente con la **componente lateral Y** de la fuerza de repulsión, evitando generar una componente longitudinal negativa que pudiera hacer que el robot intentase retroceder.

El cálculo se divide en dos partes.

#### 4.1. Repulsión lateral de todos los obstáculos

Primero recorro todas las medidas del láser:

```python
for dist, angle in laser_polar:
    if 0.05 < dist < 20:
        vector[1] -= math.sin(angle) / (dist ** 2)
```

Únicamente considero las medidas situadas entre **0.05 y 20 metros**.
Donde:

- `d`: distancia del obstáculo.
- `θ`: ángulo del obstáculo respecto al frente del robot.
- `Fy`: componente lateral de repulsión.

Esta expresión permite que:

- **Obstáculo a la izquierda:** genera una contribución negativa en Y, favoreciendo el giro hacia la derecha.
- **Obstáculo a la derecha:** genera una contribución positiva en Y, favoreciendo el giro hacia la izquierda.
- **Obstáculo frontal:** genera una contribución lateral pequeña cuando su ángulo está próximo a cero.

Además, al dividir por el cuadrado de la distancia, los obstáculos cercanos tienen una influencia considerablemente mayor que los lejanos.

La fuerza lateral total se obtiene sumando las contribuciones de todas las medidas válidas.

Esta estrategia permite tener en cuenta simultáneamente los obstáculos detectados a ambos lados del robot.

#### 4.2. Refuerzo de la evasión frontal

Uno de los problemas de utilizar únicamente la componente lateral es que un obstáculo situado casi enfrente puede generar una fuerza lateral insuficiente.

Para reforzar la evasión en estas situaciones, he incorporado una segunda componente que analiza especialmente los obstáculos frontales.

Primero defino:

```python
Kf = 20
front_angle = math.radians(45)
```

Con esto establezco una región frontal comprendida entre **-45° y +45°**.

Posteriormente, selecciono las medidas que pertenecen a esa región:

```python
front_obstacles = [
    (dist, angle) for dist, angle in laser_polar
    if 0.05 < dist < 20 and abs(angle) < front_angle
]
```

Entre ellas, selecciono la medida frontal más cercana:

```python
min_dist, min_angle = min(front_obstacles)
```

Finalmente, añado una fuerza lateral adicional dependiendo del lado en el que se encuentra dicho obstáculo:

```python
if min_angle < 0:
    vector[1] += Kf / (min_dist ** 2)

elif min_angle > 0:
    vector[1] -= Kf / (min_dist ** 2)
```

De esta manera:

- Si el obstáculo se encuentra **a la derecha**, añado una fuerza positiva que favorece la evasión hacia la izquierda.
- Si el obstáculo se encuentra **a la izquierda**, añado una fuerza negativa que favorece la evasión hacia la derecha.

La expresión de esta fuerza adicional es:

$$
F_{frontal,y}=
\begin{cases}
+\dfrac{K_f}{d_{min}^2} & \theta_{min}<0\\
-\dfrac{K_f}{d_{min}^2} & \theta_{min}>0\\
0 & \theta_{min}=0
\end{cases}
$$

Con `Kf = 20`, doy una mayor importancia al obstáculo frontal más cercano, buscando que el robot realice maniobras de evasión más pronunciadas cuando detecta un obstáculo próximo en su dirección de avance.

La fuerza resultante sigue siendo exclusivamente lateral:

```python
return vector
```

## Funcionamiento

El programa se ejecuta mediante un bucle continuo que **obtiene los datos del entorno, calcula las fuerzas y corrige la orientación del robot**.

### 1. Obtención de la posición y los datos del entorno

En cada iteración obtengo las medidas del láser:

```python
laser_data = HAL.getLaserData()
```

Posteriormente, obtengo la posición actual del robot:

```python
pos_robot = Robot_pos(
    HAL.getPose3d().x,
    HAL.getPose3d().y,
    HAL.getPose3d().yaw
)
```

También calculo la posición relativa del siguiente objetivo:

```python
target_x, target_y = pos_target(pos_robot)
```

Así dispongo de la información necesaria para calcular las fuerzas de atracción y repulsión.

### 2. Cálculo de la fuerza de atracción

La fuerza de atracción se obtiene a partir del vector que une al robot con el objetivo.

Primero calculo su módulo:

```python
magnitude_target = math.hypot(target_x, target_y)
```

Después normalizo el vector y lo multiplico por una constante de 450:

```python
target_new = [
    target_x / magnitude_target * 450,
    target_y / magnitude_target * 450
]
```

Matemáticamente:

$$
\vec F_{atraccion}=450\frac{\vec T}{\|\vec T\|}
$$

Donde `T` representa el vector hacia el objetivo.

La normalización permite que la fuerza tenga un módulo constante de **450**, independientemente de la distancia al objetivo, siempre que esta sea distinta de cero.

Con ello, la fuerza de atracción representa la dirección hacia la que quiero desplazarme, manteniendo una intensidad constante.

### 3. Ponderación de la fuerza de los obstáculos

Una vez calculado el vector de repulsión, aplico una ganancia adicional:

```python
K = 5

obstacle_new = [0, obstacle[1] * K]
```

Con ello, multiplico por cinco la componente lateral obtenida a partir del láser.

La fuerza de repulsión resultante es:

$$
\vec F_{repulsion}=
\begin{bmatrix}
0\\
5F_y
\end{bmatrix}
$$

Esta ponderación permite ajustar la importancia de los obstáculos respecto a la fuerza de atracción.

He mantenido la componente X igual a cero para que la repulsión actúe exclusivamente modificando la dirección lateral de navegación, sin introducir una fuerza longitudinal contraria al avance.

### 4. Combinación de las fuerzas virtuales

Para obtener la dirección final, combino las fuerzas de atracción y repulsión:

```python
avgForce = [
    target_new[0],
    target_new[1] + obstacle_new[1]
]
```

La fuerza resultante se calcula como:

$$
\vec F_{resultante}=
\vec F_{atraccion}+
\vec F_{repulsion}
$$

De esta manera, el vector resultante representa un compromiso entre **avanzar hacia el objetivo y desviarse de los obstáculos**.

Cuando la influencia de los obstáculos es pequeña, la dirección resultante se aproxima a la del objetivo.

Cuando aparece un obstáculo cercano, aumenta la componente lateral de repulsión y se modifica la dirección de navegación.

La expresión utilizada en el programa es:

$$
\vec F_{resultante}=
\begin{bmatrix}
450\frac{T_x}{\|\vec T\|}\\
450\frac{T_y}{\|\vec T\|}+5F_y
\end{bmatrix}
$$

### 5. Control proporcional de la velocidad angular

Una vez obtenida la fuerza resultante, calculo el ángulo que forma con el eje frontal del robot:

```python
angle = math.atan2(avgForce[1], avgForce[0])
```

Este ángulo representa la dirección hacia la que debe orientarse el robot.

Para corregir su orientación utilizo un **controlador proporcional (P)**:

```python
Kp = 7

w = Kp * angle

HAL.setW(w)
```

La velocidad angular se calcula mediante:

$$
\omega=K_p\theta
$$

Con:

$$
K_p=7
$$

El funcionamiento del controlador es el siguiente:

- **Ángulo positivo:** el robot gira hacia la izquierda.
- **Ángulo negativo:** el robot gira hacia la derecha.
- **Ángulo cercano a cero:** el robot mantiene aproximadamente su dirección de avance.

Cuanto mayor es el ángulo entre la dirección frontal y la fuerza resultante, mayor es la velocidad angular solicitada.

He establecido una ganancia proporcional elevada, buscando una **respuesta rápida ante desviaciones de la dirección deseada**.

### 6. Control de la velocidad lineal

A diferencia de la velocidad angular, la velocidad lineal se mantiene constante:

```python
HAL.setV(6)
```

Por tanto, el robot avanza con una velocidad lineal solicitada de **6 unidades de velocidad del simulador**, mientras modifica su orientación en función de las fuerzas virtuales.

Esta implementación simplifica el control, ya que la evasión de obstáculos se realiza mediante cambios en la velocidad angular, sin modificar la velocidad lineal en función de la proximidad de los obstáculos.

### 7. Detección de llegada al objetivo

Para determinar cuándo el robot ha alcanzado un objetivo, calculo la distancia euclídea entre su posición y el punto de destino:

```python
if math.hypot(target_x, target_y) < 0.5:
    currentTarget.setReached(True)
```

Cuando la distancia es inferior a **0.5 metros**, marco el objetivo como alcanzado.

Así, el sistema puede continuar con el siguiente objetivo de navegación proporcionado por `WebGUI`.

### 8. Visualización de las fuerzas

Para observar el comportamiento del algoritmo, utilizo:

```python
WebGUI.showForces(
    carForce,
    obsForce,
    avgForce_show
)
```

Previamente normalizo los vectores para representar sus direcciones.

La interfaz permite visualizar tres vectores:

- **Vector verde (`carForce`):** dirección normalizada hacia el objetivo.
- **Vector rojo (`obsForce`):** dirección normalizada de la fuerza de repulsión.
- **Vector negro (`avgForce_show`):** dirección normalizada de la fuerza resultante.

Esta representación permite analizar cómo cambia la dirección de navegación según la posición del objetivo y los obstáculos detectados.

## Análisis de los parámetros elegidos

Los parámetros utilizados en la implementación son:

| Parámetro | Valor | Función |
|---|---:|---|
| `Kf` | 20 | Refuerzo de la repulsión frontal |
| `front_angle` | 45° | Región de detección frontal |
| Distancia mínima | 0.05 m | Límite inferior de las medidas utilizadas |
| Distancia máxima | 20 m | Alcance considerado para la repulsión |
| `K` | 5 | Ganancia de la fuerza de los obstáculos |
| Atracción | 450 | Módulo de la fuerza hacia el objetivo |
| `Kp` | 7 | Ganancia del controlador angular |
| Velocidad lineal | 6 | Velocidad de avance solicitada |
| Tolerancia del objetivo | 0.5 m | Distancia para considerar alcanzado el objetivo |

### Influencia de `Kf`

El parámetro `Kf = 20` controla la intensidad del refuerzo aplicado al obstáculo frontal más cercano.

Un valor mayor provoca una componente de evasión lateral más intensa, mientras que uno menor reduce su influencia.

Su objetivo es compensar la escasa repulsión lateral que pueden generar los obstáculos situados cerca del eje frontal.

### Influencia de `K`

Con `K = 5`, multiplico la fuerza lateral de los obstáculos antes de combinarla con la atracción.

Este parámetro determina cuánto influye la evasión en la dirección final.

- Un valor mayor aumenta la desviación provocada por los obstáculos.
- Un valor menor hace que el robot priorice más directamente la dirección hacia el objetivo.

### Influencia de la fuerza de atracción

Al establecer un módulo de atracción de `450`, mantengo una referencia constante hacia el objetivo.

Esto hace que la dirección de navegación dependa del equilibrio entre esa atracción y la repulsión lateral, que varía con las medidas del sensor láser.

El valor de 450 permite dar un peso importante a la atracción, aunque la repulsión puede llegar a dominar cuando los obstáculos están suficientemente cerca.

### Influencia de `Kp`

El parámetro `Kp = 7` determina la intensidad de la corrección angular.

Por ejemplo, si el vector resultante forma un ángulo de `0.5` radianes con el eje frontal:

$$
\omega=7\cdot0.5=3.5
$$

El robot solicitará una velocidad angular de `3.5` rad/s.

Un valor mayor de `Kp` produce correcciones más intensas, mientras que uno menor genera giros más suaves.

Con este valor busco que el robot sea capaz de reaccionar rápidamente ante la aparición de obstáculos y los cambios de dirección del objetivo.

Sin embargo, una ganancia elevada también puede generar oscilaciones o movimientos bruscos, especialmente cuando el vector resultante cambia rápidamente.

### Influencia del alcance de detección

He establecido el intervalo de distancias entre `0.05` y `20` metros.

Este rango permite considerar obstáculos situados a mayor distancia, aunque su contribución individual disminuye de forma cuadrática.

Por ejemplo, manteniendo constante el ángulo, un obstáculo a 2 metros genera una contribución cuatro veces menor que otro a 1 metro.

De esta forma, los obstáculos más próximos tienen una influencia mayor en la dirección de navegación.

## Limitaciones de la implementación

Aunque la estrategia permite combinar la navegación hacia objetivos con la evasión de obstáculos, presenta algunas limitaciones importantes.

**1. Velocidad lineal constante**

El robot mantiene una velocidad lineal solicitada de `6` incluso cuando encuentra obstáculos cercanos. Por tanto, la implementación no reduce automáticamente la velocidad ante situaciones de riesgo.

**2. Ausencia de repulsión longitudinal**

Al mantener la componente X de la fuerza de repulsión en cero, el algoritmo no genera una acción de frenado o retroceso. La evasión depende de que el robot pueda modificar suficientemente su orientación.

**3. Obstáculos exactamente frontales**

Si el obstáculo más cercano está situado exactamente a 0°, el refuerzo frontal no selecciona un sentido de giro.

En una situación simétrica, también pueden cancelarse las contribuciones laterales de los distintos rayos.

**4. Posibles divisiones por cero**

Al normalizar los vectores de atracción y repulsión, el código no comprueba si sus módulos son cero.

Por ejemplo, si no existe fuerza de repulsión lateral, la normalización de `obsForce` puede provocar un error de división por cero.

**5. Posibles oscilaciones y mínimos locales**

La combinación de fuerzas no garantiza encontrar una trayectoria libre de obstáculos.

En determinadas configuraciones, el robot podría oscilar entre distintas direcciones o quedar orientado hacia una dirección que no permita avanzar hacia el objetivo.

**6. Diferencia entre coordenadas de control y visualización**

El algoritmo calcula la fuerza de atracción utilizando coordenadas relativas, mientras que `WebGUI.showLocalTarget()` recibe las coordenadas absolutas del objetivo.

Por tanto, es necesario que la interfaz interprete correctamente el sistema de referencia utilizado para mostrar el punto en la posición esperada.

## Conclusión

En esta práctica he desarrollado un algoritmo de **navegación autónoma basado en fuerzas virtuales**, combinando la atracción hacia los objetivos con la repulsión generada por los obstáculos detectados mediante un sensor láser.

La principal característica de mi implementación es que la **repulsión actúa exclusivamente sobre la componente lateral**, evitando introducir fuerzas longitudinales negativas.

Además, he incorporado un refuerzo de evasión frontal que tiene en cuenta el obstáculo más cercano dentro de una región de ±45°, buscando mejorar la respuesta ante obstáculos situados directamente en la dirección de avance.

La combinación de ambas fuerzas permite calcular una dirección resultante, que posteriormente utilizo como referencia de un **controlador proporcional de velocidad angular**, con `Kp = 7`.

La utilización de una ganancia elevada busca conseguir una respuesta rápida ante los cambios de dirección, mientras que la velocidad lineal se mantiene constante en `6`.

Por último, utilizo una tolerancia de 0.5 metros para determinar cuándo se alcanza cada objetivo.

El principal desafío de esta estrategia consiste en encontrar un equilibrio adecuado entre la **fuerza de atracción, la fuerza de repulsión y la ganancia proporcional**, para conseguir que el robot se dirija hacia sus objetivos mientras evita los obstáculos sin realizar correcciones excesivas.

La implementación constituye una base para la navegación reactiva mediante fuerzas virtuales, aunque presenta limitaciones relacionadas con la velocidad constante, la cancelación de fuerzas y la ausencia de mecanismos específicos de frenado o seguridad.


# Video de demostración

[Screencast from 2026-10-29 12-15-19.webm](https://github.com/user-attachments/assets/04ea23cd-6488-4d2e-8fbe-a2c5596c1d42)


