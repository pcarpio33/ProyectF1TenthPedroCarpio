# 🏎️ Proyecto de carrera con F1Tenth - Follow the Gap con Contador de Vueltas
**Autor:** Pedro Carpio  
**Repositorio:** https://github.com/pcarpio33/ProyectF1TenthPedroCarpio

## 📝 Descripción del Proyecto

Este proyecto implementa un **controlador reactivo** basado en el algoritmo Follow the Gap para el simulador F1Tenth, con mejoras para:
- evitar colisiones con las paredes usando **muros invisibles**
- controlar la dirección con un **controlador PD**
- adaptar la velocidad según la geometría del entorno

Además, incluye un **cronómetro de vueltas** que mide el tiempo por vuelta y cuenta el total de vueltas completadas.

> **Objetivo personal:** Desarrollar un sistema de navegación estable y rápido que permita al vehículo completar 10 vueltas sin colisiones, con adaptación en curvas y análisis de rendimiento por vueltas.

---

## 📁 Estructura del Proyecto

- [`follow_the_gap_node.py`](./follow_the_gap_node.py): Nodo ROS 2 que implementa el algoritmo Follow the Gap mejorado.
- [`lap_timer.py`](./lap_timer.py): Nodo ROS 2 que mide el tiempo por vuelta y cuenta cuántas vueltas ha dado el vehículo.

---

## ⚙️ Teoría del Algoritmo Follow the Gap

El algoritmo Follow the Gap se basa en los datos del LiDAR y consiste en:

1. **Filtrado del LiDAR:** se preprocesan las distancias para eliminar ruido.
2. **Burbuja de seguridad:** se eliminan las lecturas cercanas al obstáculo más próximo.
3. **Detección del gap:** se encuentra el hueco libre más grande.
4. **Selección del mejor punto:** se elige el punto más lejano y centrado dentro del gap.
5. **Control de dirección:** se calcula el ángulo para alcanzar ese punto.

---

## 🎯 Controlador PD y Centrado

Se usa un **controlador PD** para suavizar el ángulo de dirección:
- `P` (proporcional): responde a qué tan lejos está el punto objetivo del centro del LiDAR.
- `D` (derivativo): suaviza los cambios bruscos en el giro.

Además, se aplica un **centrado ponderado**, combinando el centro del hueco con el punto más lejano, para obtener una dirección más estable.

---

## 🏁 Contador de Vueltas

El archivo `lap_timer.py`:
- Detecta si el vehículo pasa cerca del punto inicial `(0, 0)` usando un radio de margen.
- Cuenta una nueva vuelta solo si el vehículo se alejó previamente del punto de inicio (para evitar duplicados).
- Cronometra el tiempo por vuelta y muestra el resultado por consola.

---

## Pasos para la ejecucion del proyecto
---

## 📌 Nota

Para la ejecución del siguiente proyecto es necesario tener instalados los siguientes requisitos:

### 🧩 Dependencias de ROS 2

- ROS 2 Humble (recomendado)
- `rclpy`
- `sensor_msgs`
- `nav_msgs`
- `ackermann_msgs`
- `numpy`

### 🏁 Simulador F1Tenth

- Repositorio oficial de simulación F1Tenth con `gym_bridge`
- Dependencias del simulador:
  - `gym`
  - `pygame`
  - `f1tenth_gym`
  - `f1tenth_gym_ros`

### ⚙️ Instalación sugerida

```bash
sudo apt update
sudo apt install ros-humble-ackermann-msgs ros-humble-nav-msgs ros-humble-sensor-msgs python3-numpy
```
1. aperturas de 3 terminales en el workspace
- estas nos serviran para ejecutar lo que es el simulador y los dos nodos (el contador de vueltas con cronometro y el follow the gap). 
   ![Screenshot from 2025-07-07 22-52-03](https://github.com/user-attachments/assets/77284559-696b-45da-92e2-ac343e84e19e)
2. inicio del simululador y de los nodos
- una vez ejecutadas las tres terminales podremos observar el contador de vueltas cronometrado y el simulador con la pista cargada, en el podemos observar el numero de vueltas y el tiempo ademas de la velocidad que mantiene el robot. 
![Screenshot from 2025-07-07 21-58-33](https://github.com/user-attachments/assets/20dbb7e4-826c-4735-b421-d97275af7372)

![Screenshot from 2025-07-07 21-59-21](https://github.com/user-attachments/assets/57613401-36cc-4338-b3ab-77ab7dfb2139)
3. resultados
- podemos obtener los resultados de las vueltaws y los tiempos.
![Screenshot from 2025-07-07 23-14-27](https://github.com/user-attachments/assets/2396f574-4694-412d-bb7e-75a6563771e4)

