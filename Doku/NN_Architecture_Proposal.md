# Entwurf: Neuronale Netz Architektur für RaspClaws

## 1. Zielsetzung
Das Neuronale Netz (NN) soll als "Gehirn" des Roboters fungieren. Es analysiert die Sensor-Daten (hauptsächlich Kamera) und entscheidet, welche **High-Level Action** (via `RobotControlNode`) als nächstes ausgeführt werden soll.

Dieser Ansatz nutzt die Stärken der implementierten ROS2 Actions (präzise Bewegungsausführung) und überlässt dem NN die Entscheidungsfindung ("Was tun?").

## 2. Architektur-Vorschlag

### 2.1 Eingabedaten (Input)
*   **Visuell**: Kamerabild (downscaled auf z.B. 160x120 Pixel, RGB oder Graustufen).
*   **Optional**: Status-Vektor (aktuelle Kopf-Position, zuletzt ausgeführte Action).

### 2.2 Modell-Struktur (Small CNN)
Da das Modell auf dem Raspberry Pi laufen muss, wählen wir eine effiziente Architektur:

1.  **Input Layer**: `(120, 160, 3)` [Höhe, Breite, Kanäle]
2.  **Convolutional Block 1**:
    *   Conv2D (16 Filter, 3x3 Kernel, Stride 2) -> Feature Extraction
    *   ReLU Activation
    *   BatchNorm
3.  **Convolutional Block 2**:
    *   Conv2D (32 Filter, 3x3 Kernel, Stride 2)
    *   ReLU Activation
    *   BatchNorm
4.  **Convolutional Block 3**:
    *   Conv2D (64 Filter, 3x3 Kernel, Stride 2)
    *   ReLU
5.  **Flatten**
6.  **Dense Layers (Fully Connected)**:
    *   Dense (128 Neurons) + ReLU + Dropout (0.5)
    *   Dense (64 Neurons) + ReLU
7.  **Output Layer**:
    *   Dense (N_ACTIONS) + Softmax

### 2.3 Ausgabedaten (Output Classes)
Das Netz klassifiziert die Situation und wählt eine diskrete Aktion.
Vorgeschlagene Klassen (N_ACTIONS = 6):

| ID | Klasse / Aktion | Parameter für RobotControlNode |
| :--- | :--- | :--- |
| 0 | **Forward** | `move_linear(distance_cm=10, speed=50)` |
| 1 | **Left Turn** | `rotate(angle_degrees=-20, speed=50)` |
| 2 | **Right Turn** | `rotate(angle_degrees=20, speed=50)` |
| 3 | **Look Left** | `set_head_pos(pan=current-20, ...)` |
| 4 | **Look Right** | `set_head_pos(pan=current+20, ...)` |
| 5 | **Stop / Scan** | Keine Bewegung, evtl. Kopf zentrieren |

## 3. Workflow (The Loop)

```python
while rclpy.ok():
    # 1. Sense
    image = get_latest_camera_frame()
    
    # 2. Think (Inference)
    action_idx = model.predict(image)
    
    # 3. Act
    if action_idx == 0:
        node.move_linear(10, sync=True)
    elif action_idx == 1:
        node.rotate(-20, sync=True)
    ...
    
    # Warte kurz oder verarbeite Feedback
    time.sleep(0.1)
```

## 4. Training Strategien

### A. Supervised Learning (Behavioral Cloning)
1.  Wir steuern den Roboter manuell (via GUI oder Gamepad).
2.  Wir zeichnen `(Bild, Aktion)` Paare auf.
3.  Wir trainieren das Netz offline auf dem PC.
4.  Wir deployen das Modell auf den Pi.

*Vorteil*: Einfach umzusetzen, stabile Ergebnisse für einfache Aufgaben.

### B. Reinforcement Learning (Q-Learning / PPO)
*   Der Roboter lernt durch Belohnung (z.B. "Distanz gefahren ohne Kollision").
*   Sehr rechenintensiv, Training auf Pi schwierig.

**Empfehlung**: Starten mit **Ansatz A (Behavioral Cloning)**, um die Pipeline zu testen.

## 5. Technologie-Stack
*   **Framework**: PyTorch (flexibel) oder TensorFlow Lite (schnell auf Pi).
*   **ROS2 Node**: Ein neuer `BrainNode`, der `RobotControlNode` nutzt.
