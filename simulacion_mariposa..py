import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# =================================================================
# 1. GENERACIÓN DE TRAYECTORIA MAESTRA (Escalada a 2x2m)
# =================================================================
def generate_master_butterfly(v_target=0.22, dt=0.1, scale=0.23):
    # Generación inicial con muy alta resolución para suavidad
    t_fine = np.linspace(0, 12 * np.pi, 25000)
    r = np.exp(np.sin(t_fine)) - 2 * np.cos(4 * t_fine) + np.sin((2 * t_fine - np.pi) / 24)**5
    
    # Escalado para ocupar aprox 1.85m de ancho
    x_fine = r * np.cos(t_fine) * scale
    y_fine = r * np.sin(t_fine) * scale
    
    # Re-muestreo equidistante basado en la velocidad del robot (v * dt)
    x_ref, y_ref = [x_fine[0]], [y_fine[0]]
    dist_step = v_target * dt
    accum_dist = 0
    
    for i in range(1, len(x_fine)):
        d = np.sqrt((x_fine[i]-x_fine[i-1])**2 + (y_fine[i]-y_fine[i-1])**2)
        accum_dist += d
        if accum_dist >= dist_step:
            x_ref.append(x_fine[i])
            y_ref.append(y_fine[i])
            accum_dist = 0
            
    x_ref = np.array(x_ref)
    y_ref = np.array(y_ref)
    # Cálculo de orientación (Psi) deseada
    psi_ref = np.arctan2(np.gradient(y_ref), np.gradient(x_ref))
    
    return x_ref, y_ref, psi_ref

# Parámetros Globales
dt = 0.1
v_const = 0.22 
N = 15 # Horizonte de predicción

x_ref, y_ref, psi_ref = generate_master_butterfly(v_target=v_const, scale=0.23)

# =================================================================
# 2. FUNCIÓN DE COSTO MPC
# =================================================================
def cost_function(u_flat, state, target_segment):
    u = u_flat.reshape(N, 2)
    cost = 0
    curr_x, curr_y, curr_psi = state
    
    for i in range(N):
        v, w = u[i, 0], u[i, 1]
        
        # Modelo Cinemático (Uniciclo)
        curr_x += v * np.cos(curr_psi) * dt
        curr_y += v * np.sin(curr_psi) * dt
        curr_psi += w * dt
        
        # Penalización de posición (X, Y)
        cost += 2000.0 * ((curr_x - target_segment[i, 0])**2 + (curr_y - target_segment[i, 1])**2)
        
        # Penalización de orientación (Psi)
        angle_err = np.arctan2(np.sin(curr_psi - target_segment[i, 2]), 
                                np.cos(curr_psi - target_segment[i, 2]))
        cost += 200.0 * (angle_err**2)
        
        # Penalización para mantener velocidad crucero constante
        cost += 15.0 * (v - v_const)**2
        
    return cost

# =================================================================
# 3. BUCLE DE SIMULACIÓN
# =================================================================
state = np.array([x_ref[0], y_ref[0], psi_ref[0]])
history = [state.copy()]
u_applied = [] # Aquí guardaremos los comandos para el ESP32
u_prev = np.tile([v_const, 0.0], N)

print(f"Calculando MPC para {len(x_ref)} puntos...")

for k in range(len(x_ref) - N):
    # Segmento de referencia para el horizonte actual
    target_segment = np.column_stack((x_ref[k:k+N], y_ref[k:k+N], psi_ref[k:k+N]))
    
    # Límites físicos de los motores
    bounds = [(0.0, 0.5), (-4.5, 4.5)] * N
    
    # Resolución de optimización
    res = minimize(cost_function, u_prev, args=(state, target_segment), 
                   method='SLSQP', bounds=bounds, options={'ftol': 1e-6})
    
    if res.success:
        u_prev = res.x
        v_opt, w_opt = res.x[0], res.x[1]
    else:
        v_opt, w_opt = u_prev[0], u_prev[1]

    # Guardar control para exportar
    u_applied.append([v_opt, w_opt])

    # Actualizar estado real (Simulación del movimiento)
    state[0] += v_opt * np.cos(state[2]) * dt
    state[1] += v_opt * np.sin(state[2]) * dt
    state[2] += w_opt * dt
    history.append(state.copy())

history = np.array(history)
error = np.sqrt((history[:, 0] - x_ref[:len(history)])**2 + (history[:, 1] - y_ref[:len(history)])**2)

# =================================================================
# 4. EXPORTACIÓN A ESP32 (TERMINAL)
# =================================================================
print("\n// --- COPIA DESDE AQUÍ HACIA trayectoria.h ---")
print(f"const int total_pasos = {len(u_applied)};")
v_list = [f"{u[0]:.3f}" for u in u_applied]
w_list = [f"{u[1]:.3f}" for u in u_applied]
print("const float v_ref[] PROGMEM = {" + ", ".join(v_list) + "};")
print("const float w_ref[] PROGMEM = {" + ", ".join(w_list) + "};")
print("// --- FIN DEL ARCHIVO ---")

# =================================================================
# 5. GRÁFICOS (POR SEPARADO)
# =================================================================
# Gráfico 1: Trayectoria
plt.figure(figsize=(8, 8))
plt.plot(x_ref, y_ref, 'r--', label='Referencia Ideal', alpha=0.4)
plt.plot(history[:, 0], history[:, 1], 'b-', label='Trayectoria Robot MPC', linewidth=1.5)
plt.title('Trayectoria Maestra: Mariposa de Fay (2x2m)')
plt.xlabel('Metros [X]')
plt.ylabel('Metros [Y]')
plt.axis('equal')
plt.legend()
plt.grid(True, linestyle=':')

# Gráfico 2: Error
plt.figure(figsize=(10, 4))
plt.plot(error, color='darkblue')
plt.fill_between(range(len(error)), error, color='blue', alpha=0.1)
plt.title('Error de Seguimiento Euclidiano (Precisión Milimétrica)')
plt.ylabel('Error [m]')
plt.xlabel('Pasos de tiempo (dt=0.1s)')
plt.grid(True)

plt.show()
