# ESP32-MPC-Trajectory-Tracking - JORGE ESPER - UNAB

Implementación exitosa de un controlador predictivo (MPC) en un ESP32 para el seguimiento preciso de la trayectoria "Mariposa de Fay" en un robot diferencial. Incluye modelo cinemático, discretización de Euler y sintonía de matrices de pesos Q y R para garantizar la convergencia del error.

## 1. Modelo Cinemático del Robot
El sistema se basa en la cinemática de un robot diferencial, la cual define el cambio de las coordenadas en el plano XY en función de la velocidad lineal ($v$) y angular ($\omega$):

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} \cos(\theta) & 0 \\ \sin(\theta) & 0 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} v \\ \omega \end{bmatrix}
$$

Esta matriz transforma las velocidades locales del robot en movimiento global, respetando la restricción de no-holonomía, que impide el desplazamiento lateral del vehículo.

## 2. Discretización y Predicción
Para que el ESP32 procese el control en pasos de tiempo reales ($\Delta t = 0.1s$), se utilizó el **Método de Euler** para predecir la posición estimada:

* $x_{k+1} = x_{k} + v_{k} \cos(\theta_{k}) \Delta t$
* $y_{k+1} = y_{k} + v_{k} \sin(\theta_{k}) \Delta t$
* $\theta_{k+1} = \theta_{k} + \omega_{k} \Delta t$

## 3. Lógica de Control (Función de Costo)
El MPC actúa como el cerebro del sistema, buscando las señales de control ($v$ y $\omega$) que minimizan la siguiente función de costo ($J$) en un horizonte de predicción:

* **Término Q:** Obliga al robot a mantenerse lo más cerca posible de la trayectoria de referencia.
* **Término R:** Penaliza cambios bruscos en los motores, suavizando el movimiento y protegiendo el hardware.

## 4. Implementación en Hardware (ESP32)
La ejecución física traduce las velocidades calculadas por el MPC a comandos para los motores mediante **Cinemática Inversa**:

* $v_{Derecha} = v + \frac{\omega \cdot L}{2}$
* $v_{Izquierda} = v - \frac{\omega \cdot L}{2}$

Donde **L** representa el ancho del robot. El resultado se mapea a una señal **PWM (0-255)** aplicando una función de saturación para evitar el desbordamiento en los registros del microcontrolador.

## 5. Resultados
* **Trayectoria:** Mariposa de Fay redimensionada a un área de 2x2 metros.
* **Frecuencia de control:** 10 Hz ($\Delta t = 0.1s$).
* **Hardware:** ESP32, Puente H L298N, Motores DC con reducción.
* **Precisión:** Convergencia del error de seguimiento cercana a cero mediante la sintonía fina de las matrices de pesos Q y R.

## Gráficos de Resultados

### 1. Comparativa: Simulación vs Aplicación Real
![Trayectoria Final](img/RESULTADOS%20MPC%20final.png)

### 2. Señal del Motor y Calibración de Hardware
![Señal Motor](img/RESULTADOS%20SEÑAL%20MOTOR%20Y%20HARDWARE.png)

### 3. Análisis de Error Euclidiano
![Error Euclidiano](img/GRAFICO%20ERROR%20EUCLIDIANO%20FINAL%20_%20int%2011.png)
