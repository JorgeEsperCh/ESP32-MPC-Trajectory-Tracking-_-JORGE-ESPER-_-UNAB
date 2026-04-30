# ESP32-MPC-Trajectory-Tracking - JORGE ESPER - UNAB

Implementación de un controlador predictivo (MPC) en un ESP32 para el seguimiento preciso de la trayectoria "Mariposa de Fay" en un robot diferencial. Este proyecto integra el modelado matemático, la simulación en Python y la sintonización final en hardware real.

## 1. Fundamentos Matemáticos
El sistema se basa en un modelo cinemático diferencial que define el cambio de posición $(x, y)$ y orientación ($\theta$) en función de la velocidad lineal ($v$) y angular ($\omega$):

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} \cos(\theta) & 0 \\ \sin(\theta) & 0 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} v \\ \omega \end{bmatrix}
$$

Para la ejecución en el ESP32, se utilizó la discretización por el **Método de Euler** con un paso de tiempo $\Delta t = 0.1s$:
* $x_{k+1} = x_{k} + v_{k} \cos(\theta_{k}) \Delta t$
* $y_{k+1} = y_{k} + v_{k} \sin(\theta_{k}) \Delta t$
* $\theta_{k+1} = \theta_{k} + \omega_{k} \Delta t$

## 2. Especificaciones Técnicas
* **Tiempo Total de Ejecución:** 215.2 segundos.
* **Área de Trabajo:** $2 \times 2$ metros (Escala 0.23).
* **Error Euclidiano Promedio:** ~0.025 m.
* **Hardware:** ESP32, Puente H L298N, 4x Pilas AA.

## 3. Gráficos de Resultados y Telemetría

### A. Comparativa: Simulación vs Aplicación Real
![Comparativa Final](RESULTADO%20COMPARACIÓN%20_%20SIMULACIÓN%20VS%20APLICACIÓN%20_%20CALIBRACIÓN%20FINAL.png)

### B. Análisis de Trayectoria Maestra
![Resultados MPC](RESULTADOS%20MPC%20final.png)

### C. Señal de Control vs Potencia PWM
![Control vs PWM](SEÑAL%20DE%20CONTROL%20VS%20SEÑAL%20DE%20POTENCIA%20PWM.png)

### D. Comportamiento de Motores y Hardware
![Señal Motor](RESULTADOS%20SEÑAL%20MOTOR%20Y%20HARDWARE.png)

### E. Análisis de Error de Seguimiento (Intento 11)
![Error Euclidiano](GRAFICO%20ERROR%20EUCLIDIANO%20FINAL%20_%20int%2011.png)
