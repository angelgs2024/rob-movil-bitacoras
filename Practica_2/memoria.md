# Visual Follow Line

## Objetivo

El objetivo de esta práctica es implementar un algoritmo de **seguimiento visual de una línea roja** para que un robot sea capaz de avanzar por un recorrido y corregir su trayectoria de forma autónoma.

Para conseguirlo, utilizo la imagen capturada por la cámara y un **controlador proporcional-derivativo (PD)**. A partir de la posición de la línea en la imagen, calculo el error respecto a un punto de referencia y modifico la **velocidad angular** del robot para tratar de mantener la línea centrada. Además, adapto la **velocidad lineal** en función del error: el robot puede avanzar más rápido cuando está alineado y reduce su velocidad cuando necesita realizar una corrección mayor.

## Componentes

Para esta práctica tenemos disponible un robot con:

- **Cámara:** permite obtener imágenes del entorno y detectar la línea roja que debemos seguir.
- **Sistema de movimiento:** permite modificar la velocidad lineal (`HAL.setV()`) y la velocidad angular (`HAL.setW()`).
- **Interfaz de visualización:** mediante `WebGUI.showImage()` podemos observar la imagen procesada y el punto detectado.

La implementación se realiza en **Python**, utilizando principalmente **OpenCV** para el procesamiento de imagen y **NumPy** para definir los rangos de color.

## Método auxiliar: `detect_red(image)`

Para simplificar el programa, he creado una función que recibe la imagen capturada por la cámara y devuelve las coordenadas **`(cx, cy)` del centroide de los píxeles detectados como rojos**.

### 1. Conversión de BGR a HSV

Lo primero que hago es convertir la imagen del espacio de color **BGR**, con el que trabaja OpenCV habitualmente, al espacio **HSV**:

```python
image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
```

HSV separa el **tono** (`H`), la **saturación** (`S`) y el **valor o brillo** (`V`), lo que facilita la segmentación por color.

### 2. Detección del color rojo

En OpenCV, el tono `H` se representa normalmente entre `0` y `179`. Como el rojo se encuentra en los dos extremos de ese intervalo, utilizo **dos rangos**:

```python
boundaries = [
    ([0, 100, 100], [10, 255, 255]),
    ([170, 100, 100], [179, 255, 255])
]

mask_h = cv2.inRange(
    image_hsv,
    np.array([0, 100, 100]),
    np.array([10, 255, 255])
)

mask_v = cv2.inRange(
    image_hsv,
    np.array([170, 100, 100]),
    np.array([179, 255, 255])
)

mask = cv2.bitwise_or(mask_h, mask_v)
```

Cada máscara es una imagen binaria: los píxeles que cumplen el rango aparecen en blanco y el resto en negro. Después uno ambas máscaras mediante una **operación OR**, de modo que detecto los rojos situados en cualquiera de los dos intervalos.

Los límites mínimos de saturación y brillo (`S ≥ 100` y `V ≥ 100`) ayudan a descartar colores poco saturados o zonas demasiado oscuras. **No obstante, la variable `mask_v` no es una máscara independiente de brillo:** en el código representa el segundo intervalo de tono rojo.

### 3. Cálculo del centroide

Una vez obtenida la máscara, utilizo los **momentos de imagen** para calcular el centroide del conjunto de píxeles detectados:

```python
M = cv2.moments(mask)

if M["m00"] != 0:
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
```

El momento `m00` representa el área ponderada de la máscara; `m10` y `m01` permiten calcular las coordenadas del centroide:

Compruebo que `m00` sea distinto de cero para evitar una división por cero cuando no se detecta ningún píxel rojo. Por último, marco el punto calculado sobre la imagen procesada y la muestro:

```python
cv2.circle(output, (cx, cy), 5, (255, 0, 0), -1)
WebGUI.showImage(output)

return (cx, cy)
```

En el seguimiento utilizo **únicamente `cx`**, ya que el control se basa en el desplazamiento horizontal de la línea respecto al punto objetivo. La coordenada `cy` se calcula, pero no interviene en la corrección.

## Funcionamiento

El programa se ejecuta en un bucle continuo que **captura, detecta, calcula y corrige**:

### 1. Definición del punto objetivo

```python
point_objetivo = 318
error = 0
error_anterior = 0
```

Establezco el **punto objetivo en `x = 318`**, que representa la posición horizontal en la que quiero mantener el centroide de la línea. Valor obtenido imprimiendo el punto central con el coche en parado.

En cada iteración calculo el error:

```python
error = point_x - point_objetivo
```

- **Error igual a cero:** la línea está situada en el punto objetivo.
- **Error positivo:** el centroide se encuentra a la derecha de dicho punto.
- **Error negativo:** el centroide se encuentra a la izquierda.

El objetivo del controlador es hacer que ese error tienda a cero.

### 2. Control PD de la velocidad angular

Para corregir la orientación del robot, utilizo un **controlador proporcional-derivativo (PD)**. Este controlador combina dos acciones:

- **Proporcional (P):** corrige en función de la magnitud del error actual. Cuanto más separada esté la línea del punto objetivo, mayor será la corrección solicitada.
- **Derivativa (D):** responde a los cambios del error entre iteraciones. Permite anticipar la evolución del desplazamiento y puede amortiguar las oscilaciones, siempre que su ganancia esté bien ajustada.

La implementación es:

```python
d_error = error - error_anterior

Kp = 0.01
Kd = 0.06

u = -Kp * error - Kd * d_error
HAL.setW(u)
```

Donde:

| Parámetro | Valor | Función |
| --- | ---: | --- |
| `Kp` | `0.01` | Determina la intensidad de la corrección proporcional. |
| `Kd` | `0.06` | Determina la respuesta ante las variaciones del error. |
| `error` | Variable | Desplazamiento horizontal actual respecto al objetivo, en píxeles. |
| `d_error` | Variable | Diferencia del error respecto a la iteración anterior. |
| `u` | Variable | Orden enviada como velocidad angular mediante `HAL.setW()`. |

Los signos negativos forman parte de la convención de giro utilizada en este código. **Su efecto físico (izquierda o derecha) depende de la convención de signos de `HAL.setW()`** y debe comprobarse en el simulador.

**Importante:** en este programa el término derivativo es una **diferencia entre muestras**, sin dividir por el tiempo transcurrido. Por tanto, el comportamiento de `Kd` depende también de la frecuencia a la que se ejecuta el bucle. `Frequency.tick()` ayuda a regular la ejecución, pero no se calcula explícitamente un `Δt`.

### 3. Ajuste de la velocidad lineal

Además de corregir la orientación, adapto la velocidad de avance al error horizontal:

```python
Kv = 0.04
V = 16 - Kv * abs(error)
HAL.setV(V)
```

Con esta estrategia:

- Cuando el robot está alineado, el error es pequeño y puede avanzar a mayor velocidad.
- Cuando la línea aparece desplazada, la velocidad lineal disminuye para facilitar la corrección de la trayectoria.
- Utilizo el valor absoluto del error porque la reducción de velocidad debe ser la misma tanto si la línea está a la izquierda como a la derecha.

Este ajuste **no forma parte del PD angular**: es una regulación proporcional independiente de la velocidad lineal. Las unidades físicas de `V` y `u` dependen de la interfaz del simulador; aquí analizo los valores enviados por el código sin asumir una conversión a m/s o rad/s.


### 4. Búsqueda y recuperación de la línea

Para mejorar la autonomía del robot, he añadido un sistema que permite **buscar la línea roja cuando no se encuentra dentro del campo de visión de la cámara**. De esta manera, el robot no depende de detectar la línea desde el inicio y puede intentar recuperarla si la pierde durante el recorrido.

Para ello, utilizo las siguientes variables:

```python
linea_encontrada = False
ultima_direccion = 1
direccion_busqueda = 1
contador_busqueda = 0
```

- **`linea_encontrada`:** indica si el robot ha detectado la línea al menos una vez.
- **`ultima_direccion`:** almacena el sentido de giro hacia el que debe intentar recuperar la línea.
- **`direccion_busqueda`:** determina hacia qué lado gira durante la búsqueda inicial.
- **`contador_busqueda`:** permite cambiar periódicamente el sentido de giro.

El algoritmo distingue tres situaciones:

#### 1. Búsqueda inicial (`SEARCH_LINE`)

Si el robot todavía no ha encontrado la línea, comienza a avanzar lentamente con una velocidad lineal de `2`, mientras realiza giros alternos.

```python
HAL.setV(2)

if not linea_encontrada:
    HAL.setW(0.2 * direccion_busqueda)

    contador_busqueda += 1

    if contador_busqueda >= 60:
        direccion_busqueda *= -1
        contador_busqueda = 0
```

La velocidad angular se establece en `0.2`, buscando que el robot realice correcciones suaves mientras avanza.

Cada **60 iteraciones** se invierte el sentido de giro multiplicando `direccion_busqueda` por `-1`, generando una trayectoria de búsqueda en zigzag.

Con este comportamiento, el robot intenta explorar diferentes direcciones hasta detectar la línea roja.

#### 2. Recuperación de la línea (`RECOVERY`)

Si el robot ya había detectado la línea anteriormente, pero deja de verla, utiliza la última dirección conocida para intentar recuperarla.

```python
else:
    HAL.setW(0.3 * ultima_direccion)
```

En esta situación mantiene una velocidad lineal de `2` y utiliza una velocidad angular de `0.3`, ligeramente superior a la empleada en la búsqueda inicial, para intentar regresar hacia la línea.

La variable `ultima_direccion` se actualiza durante el seguimiento en función de la posición de la línea respecto al punto objetivo:

```python
if abs(error) > 15:
    ultima_direccion = -1 if error > 0 else 1
```

De esta manera, el robot recuerda el último sentido de corrección utilizado cuando la línea estaba suficientemente desplazada del centro.

#### 3. Retorno al seguimiento (`FOLLOW_LINE`)

Cuando la cámara vuelve a detectar la línea roja, el robot abandona automáticamente el comportamiento de búsqueda y vuelve a utilizar el controlador PD para corregir su trayectoria.

Además, se actualizan las variables:

```python
linea_encontrada = True
siguiendo = True
contador_busqueda = 0
```

La variable `siguiendo` permite evitar un cambio brusco en la componente derivativa cuando se recupera la línea, ya que en la primera iteración después de la búsqueda se establece `d_error = 0`.

De esta manera, el sistema recuerda que la línea ya ha sido localizada y reinicia el contador utilizado durante la búsqueda inicial.

Este mecanismo permite que el robot tenga un comportamiento más autónomo, al combinar el **seguimiento visual mediante un controlador PD** con un procedimiento de **búsqueda y recuperación de la trayectoria**.

No obstante, la búsqueda no garantiza encontrar siempre la línea, ya que depende de que esta llegue a aparecer dentro del campo de visión de la cámara.


## Análisis de los parámetros elegidos

Los parámetros finales del código son `Kp = 0.01`, `Kd = 0.06`, `Kv = 0.04` y una velocidad lineal de referencia de `16`.

### Influencia de `Kp` y `Kd`

Con un valor de **`Kp = 0.01`**, la acción proporcional es relativamente suave: por ejemplo, un error de `50` píxeles genera una contribución angular de `-0.5`.

La ganancia **`Kd = 0.06`** hace que el controlador responda con mayor intensidad a variaciones rápidas del error. Por ejemplo, si el error pasa de `40` a `50` píxeles en una iteración:

Sin acción derivativa, la orden habría sido `-0.5`. La derivada aumenta la corrección en este caso porque el error está creciendo. Si el error estuviera disminuyendo, el término derivativo actuaría en sentido contrario a la componente proporcional y podría contribuir a amortiguar el giro.



## Conclusión

En esta práctica he implementado un sistema de **seguimiento visual de una línea roja** que combina visión artificial y control de movimiento. Mediante OpenCV identifico los píxeles rojos de la imagen y calculo su centroide; después utilizo un **PD para ajustar la velocidad angular** según el error horizontal y su variación entre iteraciones.

Además, he añadido una regulación de la **velocidad lineal en función del error**, de modo que el robot reduce su avance cuando necesita corregir más su orientación. El aspecto más importante del ajuste es encontrar un equilibrio entre **rapidez de corrección y estabilidad**, especialmente en la elección de `Kd`, porque una ganancia derivativa demasiado alta puede amplificar pequeñas variaciones de la detección y provocar oscilaciones.

La estructura propuesta es sencilla y permite seguir desarrollando el sistema mediante mejoras en la detección, el tratamiento de la pérdida de línea y la limitación de velocidades. **Con el código disponible se puede analizar el funcionamiento esperado, pero no cuantificar el rendimiento real sin registros o pruebas de ejecución.**

## Vídeo de demostración

<!-- Añadir aquí el vídeo de la práctica o su enlace a GitHub cuando esté disponible. -->
