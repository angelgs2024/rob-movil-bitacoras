
# Visual Follow Line

## Objetivo

El objetivo de esta práctica es implementar un algoritmo de **seguimiento visual de una línea roja** para que un robot sea capaz de avanzar por un recorrido y corregir su trayectoria de forma autónoma.

Para conseguirlo, utilizo la imagen capturada por la cámara y un **controlador proporcional-derivativo (PD) con filtrado de la componente derivativa**. A partir de la posición de la línea en la imagen, calculo el error respecto a un punto de referencia y modifico la **velocidad angular** del robot para mantener la línea centrada.

Además, adapto la **velocidad lineal** en función del error: el robot avanza más rápido cuando está alineado y reduce su velocidad cuando necesita realizar correcciones mayores.

## Componentes

Para esta práctica tenemos disponible un robot con:

- **Cámara:** permite obtener imágenes del entorno y detectar la línea roja que debemos seguir.
- **Sistema de movimiento:** permite modificar la velocidad lineal (`HAL.setV()`) y la velocidad angular (`HAL.setW()`).
- **Interfaz de visualización:** mediante `WebGUI.showImage()` podemos observar la imagen procesada y el punto detectado.

La implementación se realiza en **Python**, utilizando principalmente **OpenCV** para el procesamiento de imágenes y **NumPy** para definir los rangos de color.

## Método auxiliar: `detect_red(image)`

Para simplificar el programa, he creado una función que recibe la imagen capturada por la cámara y devuelve las coordenadas **`(cx, cy)` del centroide de los píxeles detectados como rojos**.

### 1. Conversión de BGR a HSV

Lo primero que hago es convertir la imagen del espacio de color **BGR**, utilizado habitualmente por OpenCV, al espacio **HSV**:

```python
image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
```

HSV separa el **tono (H)**, la **saturación (S)** y el **valor o brillo (V)**, lo que facilita la detección de colores ante ciertas variaciones de iluminación.

### 2. Detección del color rojo

En OpenCV, el tono `H` se representa normalmente entre `0` y `179`. Como el rojo se encuentra en los dos extremos de ese intervalo, utilizo **dos rangos**:

```python
boundaries = [
    ([0, 100, 100], [10, 255, 255]),
    ([170, 100, 100], [179, 255, 255])
]
```

Para detectar los píxeles correspondientes a ambos intervalos, creo dos máscaras utilizando `cv2.inRange()`:

```python
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
```

- **`mask_h`:** identifica los píxeles del primer intervalo de rojo.
- **`mask_v`:** identifica los píxeles del segundo intervalo de rojo.

Posteriormente, combino ambas máscaras:

```python
mask = cv2.bitwise_or(mask_h, mask_v)
```

Cada máscara es una imagen binaria en la que los píxeles que cumplen los requisitos aparecen en blanco y el resto en negro.

Mediante la operación **OR**, obtengo una única máscara que contiene los píxeles rojos detectados en cualquiera de los dos intervalos.

Los límites mínimos de saturación y brillo (`S ≥ 100` y `V ≥ 100`) permiten descartar colores poco saturados y regiones demasiado oscuras.

Aunque en el código aparece un comentario que relaciona `mask_v` con el brillo, realmente esta máscara corresponde al segundo intervalo del tono rojo, no a un filtrado independiente de iluminación.

### 3. Cálculo del centroide

Una vez obtenida la máscara, utilizo los **momentos de imagen** para calcular el centroide del conjunto de píxeles detectados:

```python
M = cv2.moments(mask)

if M["m00"] != 0:
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
```

El momento `m00` representa la suma de intensidades de la máscara, que en este caso permite obtener su área ponderada. Los momentos `m10` y `m01` permiten calcular la posición del centroide.

Compruebo que `m00` sea distinto de cero para evitar una división por cero cuando no se detecta ningún píxel rojo.

Finalmente, marco el centroide calculado mediante `cv2.circle()` y muestro la imagen procesada con `WebGUI.showImage()`.

```python
cv2.circle(output, (cx, cy), 5, (255, 0, 0), -1)
WebGUI.showImage(output)

return (cx, cy)
```

La función devuelve:

- **`(cx, cy)`** cuando detecta la línea.
- **`(None, None)`** cuando no encuentra píxeles rojos.

En el seguimiento utilizo **únicamente `cx`**, ya que el control se basa en el desplazamiento horizontal de la línea respecto al punto objetivo.

La coordenada `cy` se calcula, pero no interviene en la corrección.

En la implementación actual, el bucle que recorre `boundaries` es redundante, porque las dos máscaras ya se calculan y combinan antes de recorrerlo. Esto no cambia el centroide obtenido, pero provoca operaciones repetidas cuando no se detecta la línea.

## Funcionamiento

El programa se ejecuta en un bucle continuo que **captura, detecta, calcula y corrige**.

### 1. Definición del punto objetivo

Inicialmente establezco:

```python
point_objetivo = 318
error = 0
error_anterior = 0
d_error_filtrado = 0.0
```

El **punto objetivo se sitúa en `x = 318`**, que representa la posición horizontal en la que quiero mantener el centroide de la línea. Este valor se obtiene comprobando la posición de referencia con el coche parado.

En cada iteración calculo el error:

```python
error = point_x - point_objetivo
```

Pueden darse tres situaciones:

- **Error igual a cero:** la línea está situada en el punto objetivo.
- **Error positivo:** el centroide se encuentra a la derecha del punto objetivo.
- **Error negativo:** el centroide se encuentra a la izquierda.

El objetivo del controlador es conseguir que este error tienda a cero.

### 2. Control PD de la velocidad angular

Para corregir la orientación del robot, utilizo un **controlador proporcional-derivativo (PD)**.

Este controlador combina dos acciones:

- **Proporcional (P):** corrige en función de la magnitud del error actual. Cuanto más separada esté la línea del punto objetivo, mayor será la corrección solicitada.
- **Derivativa (D):** responde a las variaciones del error entre iteraciones. Permite tener en cuenta cómo está evolucionando el desplazamiento y puede ayudar a amortiguar las oscilaciones.

Los parámetros utilizados son:

| Parámetro | Valor | Función |
|---|---|---|
| `Kp` | `0.008` | Ganancia de la componente proporcional. |
| `Kd` | `0.06` | Ganancia de la componente derivativa filtrada. |
| `error` | Variable | Desplazamiento horizontal respecto al objetivo. |
| `d_error_filtrado` | Variable | Variación del error suavizada mediante un filtro. |
| `u` | Variable | Orden de velocidad angular enviada al robot. |

La velocidad angular se calcula mediante:

```python
Kp = 0.008
Kd = 0.06

u = -Kp * error - Kd * d_error_filtrado

HAL.setW(u)
```

#### Filtrado de la componente derivativa

Una de las modificaciones incorporadas consiste en **filtrar la componente derivativa antes de utilizarla en el controlador**.

Primero calculo la variación del error entre dos iteraciones consecutivas:

```python
d_error = error - error_anterior
```

Posteriormente, aplico un filtro exponencial:

```python
d_error_filtrado = 0.75 * d_error_filtrado + 0.25 * d_error
```

Este filtro combina:

- **75 % del valor filtrado anterior**, manteniendo parte de la información de las iteraciones previas.
- **25 % de la nueva variación del error**, incorporando los cambios recientes.

El objetivo es **reducir la sensibilidad del controlador derivativo a cambios bruscos en la detección de la línea**.

**Importante:** el término derivativo se calcula como una diferencia entre muestras, sin dividir por el tiempo transcurrido. Por tanto, su comportamiento depende también de la frecuencia de ejecución del programa, regulada mediante `Frequency.tick()`.

### 3. Ajuste de la velocidad lineal

Además de corregir la orientación, adapto la velocidad de avance en función del error horizontal detectado.

Para ello, utilizo:

```python
Kv = 0.055

V = 18.5 - Kv * abs(error)

HAL.setV(V)
```

Con esta estrategia:

- **Cuando el robot está alineado**, el error es pequeño y puede avanzar a una velocidad cercana a `18.5`.
- **Cuando la línea aparece desplazada**, la velocidad lineal disminuye para facilitar la corrección de la trayectoria.
- **Cuando el error se reduce**, el robot vuelve a aumentar progresivamente su velocidad.

Utilizo el valor absoluto del error porque quiero reducir la velocidad independientemente de que la línea se encuentre a la izquierda o a la derecha.

Por ejemplo:

| Error absoluto (píxeles) | Velocidad lineal |
|---:|---:|
| 0 | 18.50 |
| 25 | 17.125 |
| 50 | 15.75 |
| 100 | 13.00 |
| 150 | 10.25 |
| 200 | 7.50 |

He aumentado la velocidad de referencia hasta **`18.5`**, buscando que el robot aproveche mejor los tramos en los que la línea está centrada.

Al mismo tiempo, con **`Kv = 0.055`**, la velocidad disminuye conforme aumenta el error, buscando un equilibrio entre rapidez y capacidad de corrección en las curvas.

Este ajuste **no forma parte del controlador PD angular**, sino que constituye una regulación proporcional independiente de la velocidad lineal.

Cabe destacar que el programa no establece explícitamente una velocidad mínima ni máxima mediante saturación. Por tanto, los valores calculados se envían directamente a `HAL.setV()`.

### 4. Búsqueda y recuperación de la línea

Para mejorar la autonomía del robot, he añadido un sistema que permite **buscar la línea roja cuando no se encuentra dentro del campo de visión de la cámara**.

De esta manera, el robot no depende de detectar la línea desde el inicio y puede intentar recuperarla si la pierde durante el recorrido.

Para ello, utilizo las siguientes variables:

```python
linea_encontrada = False
ultima_direccion = 1
direccion_busqueda = 1
contador_busqueda = 0
```

Sus funciones son:

- **`linea_encontrada`:** indica si el robot ha detectado la línea al menos una vez.
- **`ultima_direccion`:** almacena el último sentido de giro registrado para intentar recuperar la línea.
- **`direccion_busqueda`:** determina hacia qué lado gira durante la búsqueda inicial.
- **`contador_busqueda`:** permite cambiar periódicamente el sentido de giro.

El algoritmo distingue tres situaciones.

#### 1. Búsqueda inicial (`SEARCH_LINE`)

Si el robot todavía no ha encontrado la línea, comienza a avanzar lentamente con una velocidad lineal de `2` mientras realiza giros alternos.

Para ello, establece:

```python
HAL.setV(2)

if not linea_encontrada:
    HAL.setW(0.2 * direccion_busqueda)

    contador_busqueda += 1

    if contador_busqueda >= 60:
        direccion_busqueda *= -1
        contador_busqueda = 0
```

La velocidad angular tiene una magnitud de `0.2`, buscando que el robot realice correcciones suaves mientras avanza.

En cada iteración aumenta `contador_busqueda` y, cuando alcanza **60 iteraciones**, invierte el sentido de giro.

Posteriormente, reinicia el contador a cero.

Así, el robot alterna periódicamente entre giros a izquierda y derecha, intentando localizar la línea.

Este procedimiento genera una búsqueda con cambios de dirección, aunque no garantiza encontrar la trayectoria.

#### 2. Recuperación de la línea (`RECOVERY`)

Si el robot ya había detectado la línea anteriormente, pero deja de verla, utiliza la última dirección conocida para intentar recuperarla.

En esta situación mantiene:

- **Velocidad lineal:** `2`.
- **Velocidad angular:** `0.3 * ultima_direccion`.

```python
else:
    HAL.setW(0.3 * ultima_direccion)
```

La velocidad angular tiene una magnitud de `0.3`, ligeramente superior a la utilizada durante la búsqueda inicial.

La variable `ultima_direccion` se actualiza mientras el robot sigue la línea, siempre que el error sea suficientemente grande:

```python
if abs(error) > 15:
    ultima_direccion = -1 if error > 0 else 1
```

De esta manera:

- Si el error es positivo y supera `15` píxeles, se almacena `-1`.
- Si el error es negativo y su magnitud supera `15` píxeles, se almacena `1`.
- Si el error absoluto es menor o igual a `15`, se conserva la dirección anterior.

Así, el robot recuerda el último sentido de corrección registrado cuando la línea estaba suficientemente desplazada del centro.

Cuando pierde la referencia visual, utiliza ese sentido para intentar volver a encontrarla.

#### 3. Retorno al seguimiento (`FOLLOW_LINE`)

Cuando la cámara vuelve a detectar la línea roja, el robot abandona automáticamente el comportamiento de búsqueda y vuelve a utilizar el controlador PD para corregir su trayectoria.

Además, actualiza las variables:

```python
linea_encontrada = True
contador_busqueda = 0
```

De esta manera, el sistema recuerda que la línea ya ha sido localizada y reinicia el contador utilizado durante la búsqueda inicial.

**Es importante señalar que el error anterior y el valor derivativo filtrado no se reinician al perder o recuperar la línea.**

El mecanismo permite combinar el **seguimiento visual mediante un controlador PD** con un procedimiento de **búsqueda y recuperación de la trayectoria**.

No obstante, la recuperación no está garantizada, ya que depende de que la línea vuelva a entrar en el campo de visión de la cámara.

## Análisis de los parámetros elegidos

Los parámetros utilizados en la versión actual son:

| Parámetro | Valor |
|---|---:|
| `Kp` | `0.008` |
| `Kd` | `0.06` |
| `Kv` | `0.055` |
| Velocidad lineal de referencia | `18.5` |
| Peso de la derivada anterior | `0.75` |
| Peso de la derivada actual | `0.25` |
| Punto objetivo | `318` |

### Influencia de `Kp` y `Kd`

Con un valor de **`Kp = 0.008`**, la componente proporcional genera una corrección angular de `-0.4` cuando el error es de `50` píxeles.

Por otro lado, **`Kd = 0.06`** hace que el controlador responda a las variaciones del error, utilizando la señal previamente filtrada.

Por ejemplo, supongamos que el error pasa de `40` a `50` píxeles entre dos iteraciones y que el valor derivativo filtrado anterior era cero.

La variación del error es:

```python
d_error = 50 - 40
d_error = 10
```

Al aplicar el filtro:

```python
d_error_filtrado = 0.75 * 0 + 0.25 * 10
d_error_filtrado = 2.5
```

Por tanto, la velocidad angular será:

```python
u = -0.008 * 50 - 0.06 * 2.5

u = -0.4 - 0.15

u = -0.55
```

Sin componente derivativa, la velocidad angular solicitada habría sido `-0.4`.

La acción derivativa aumenta la corrección en este ejemplo porque el error está creciendo. Si el error estuviera disminuyendo, la componente derivativa podría actuar en sentido contrario a la proporcional y contribuir a amortiguar el giro.

El filtrado permite que la respuesta a estas variaciones sea menos brusca que si utilizara directamente la diferencia de errores sin suavizar.

### Influencia de `Kv` y de la velocidad de referencia

La velocidad lineal se determina mediante:

```python
V = 18.5 - 0.055 * abs(error)
```

El valor de referencia `18.5` representa la velocidad solicitada cuando el error es cero.

Por su parte, `Kv = 0.055` determina cuánto disminuye esa velocidad cuando aumenta el desplazamiento de la línea.

Con esta combinación, busco:

- **Aumentar la velocidad en las rectas**, aprovechando los momentos en los que la línea está centrada.
- **Reducir progresivamente la velocidad en las curvas**, cuando el desplazamiento horizontal de la línea aumenta.
- **Mantener un compromiso entre rapidez y estabilidad**, evitando que el robot avance siempre a una velocidad constante independientemente del error.

Sin embargo, el error horizontal no representa directamente la curvatura de la trayectoria. Por tanto, esta estrategia reduce la velocidad según la posición aparente de la línea, pero no anticipa necesariamente todas las curvas.

## Conclusión

En esta práctica he implementado un sistema de **seguimiento visual de una línea roja** que combina visión artificial y control de movimiento.

Mediante OpenCV identifico los píxeles rojos de la imagen y calculo su centroide. Posteriormente, utilizo un **controlador PD con filtrado de la componente derivativa** para ajustar la velocidad angular según el error horizontal y su evolución entre iteraciones.

La incorporación del filtrado permite suavizar las variaciones utilizadas por el término derivativo, buscando reducir las oscilaciones que pueden producirse cuando cambia bruscamente la posición detectada de la línea.

Además, he ajustado la **velocidad lineal en función del error**, utilizando una velocidad de referencia de `18.5` y un coeficiente de reducción `Kv = 0.055`. Con ello, pretendo aumentar la rapidez en los tramos bien alineados y disminuir el avance cuando se requieren correcciones mayores.

También he incorporado un mecanismo de **búsqueda inicial y recuperación de la línea**, que permite al robot intentar recuperar la trayectoria cuando deja de detectar la referencia visual.

El principal desafío de este sistema consiste en encontrar un equilibrio entre **velocidad, capacidad de corrección y estabilidad**, especialmente al ajustar las ganancias del controlador y la reducción de velocidad.


## Vídeo de demostración

https://github.com/user-attachments/assets/b1e107ec-87dd-43cc-ab7a-4e4f2abdc617

## Video de demostración Nurburgring

[Screencast from 2026-10-15 22-47-50.webm](https://github.com/user-attachments/assets/51866c6b-1d61-4d77-85ee-40602ed59686)

