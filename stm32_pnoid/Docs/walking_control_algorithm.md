# Lý thuyết toán học điều khiển đi bộ Humanoid — PNOID

## Mục lục
1. [Ký hiệu và quy ước](#1-ký-hiệu-và-quy-ước)
2. [Mô hình hình học](#2-mô-hình-hình-học)
3. [Forward Kinematics — Denavit-Hartenberg](#3-forward-kinematics)
4. [Inverse Kinematics — Giải tích](#4-inverse-kinematics)
5. [Sinh quỹ đạo bước đi](#5-sinh-quỹ-đạo-bước-đi)
6. [Zero Moment Point (ZMP)](#6-zero-moment-point)
7. [Ước lượng trạng thái — Sensor Fusion](#7-ước-lượng-trạng-thái)
8. [Bộ điều khiển ổn định](#8-bộ-điều-khiển-ổn-định)
   - 8.1 [Phương trình Euler-Lagrange](#81-phương-trình-euler-lagrange-cho-cân-bằng)
   - 8.2 [Mô hình không gian trạng thái](#82-mô-hình-không-gian-trạng-thái)
   - 8.3 [Feedforward từ động lực học](#83-feedforward-từ-động-lực-học)
   - 8.4 [LQR — Linear Quadratic Regulator](#84-lqr--linear-quadratic-regulator)
   - 8.5 [LQI — LQR with Integral Action](#85-lqi--lqr-with-integral-action)
   - 8.6 [Phân tích ổn định — Lyapunov](#86-phân-tích-ổn-định--lyapunov)
   - 8.7 [MPC — Model Predictive Control](#87-mpc--model-predictive-control)
   - 8.8 [Giải QP trên STM32 — ADMM](#88-giải-qp-trên-stm32--admm)
   - 8.9 [Adaptive Control — Bù sai số mô hình](#89-adaptive-control--bù-sai-số-mô-hình)
   - 8.10 [Gain Scheduling theo Gait Phase](#810-gain-scheduling-theo-gait-phase)
   - 8.11 [Control Allocation](#811-phân-bổ-lực--control-allocation)
   - 8.12 [Tổng hợp góc khớp cuối cùng](#812-tổng-hợp-góc-khớp-cuối-cùng)
   - 8.13 [Tổng quan pipeline điều khiển](#813-tổng-quan-pipeline-điều-khiển)
9. [Phản hồi vị trí servo](#9-phản-hồi-vị-trí-servo)
10. [Mô hình con lắc ngược (LIPM)](#10-mô-hình-con-lắc-ngược)
11. [Tham số và điều kiện biên](#11-tham-số-và-điều-kiện-biên)

---

## 1. Ký hiệu và quy ước

### 1.1 Hệ tọa độ

Gốc O đặt tại **tâm pelvis** (điểm giữa hai khớp HipYaw):

- **X** : hướng trước mặt robot (sagittal, forward)
- **Y** : hướng sang trái robot (lateral, left)
- **Z** : hướng lên trên (vertical, up)

Quy tắc bàn tay phải. Góc dương theo chiều ngược kim đồng hồ khi nhìn từ đầu dương trục quay.

### 1.2 Ký hiệu chung

| Ký hiệu | Ý nghĩa |
|----------|---------|
| θᵢ | Góc khớp thứ i (rad) |
| L₁ | Chiều dài đùi (thigh), HipPitch → Knee |
| L₂ | Chiều dài ống chân (shank), Knee → Ankle |
| d | Khoảng cách ngang từ tâm pelvis đến HipYaw (hip half-width) |
| h₀ | Chiều cao từ ankle đến mặt đất (foot height) |
| h_z | Chiều cao từ pelvis xuống trục HipRoll |
| g | Gia tốc trọng trường, 9.81 m/s² |
| m | Khối lượng robot (kg) |
| T | Chu kỳ 1 stride = 2 bước (s) |
| Δt | Chu kỳ điều khiển (s) |

### 1.3 Ký hiệu khớp

Mỗi chân có 6 khớp, đánh số từ gốc (pelvis) ra ngoài (bàn chân):

| Index | Tên | Trục quay | Ký hiệu |
|-------|-----|-----------|---------|
| 1 | HipYaw | Z | θ₁ |
| 2 | HipRoll | X | θ₂ |
| 3 | HipPitch | Y | θ₃ |
| 4 | KneePitch | Y | θ₄ |
| 5 | AnklePitch | Y | θ₅ |
| 6 | AnkleRoll | X | θ₆ |

Thân trên (torso):

| Index | Tên | Trục quay | Ký hiệu |
|-------|-----|-----------|---------|
| T1 | TorsoYaw | Z | θ_T1 |
| T2 | TorsoRoll | X | θ_T2 |

---

## 2. Mô hình hình học

### 2.1 Chuỗi động học

Từ pelvis đến bàn chân là chuỗi nối tiếp 6 khớp quay:

```
                    Z
                    ↑
          Pelvis ───┼──► X
            │
         ── θ₁ ── (quay quanh Z)
            │
         ── θ₂ ── (quay quanh X)
            │
         ── θ₃ ── (quay quanh Y)
            │
           [L₁]   ← thanh đùi
            │
         ── θ₄ ── (quay quanh Y)
            │
           [L₂]   ← thanh ống chân
            │
         ── θ₅ ── (quay quanh Y)
            │
         ── θ₆ ── (quay quanh X)
            │
          Foot ──── mặt đất
```

### 2.2 Khoảng cách offset ban đầu

Từ tâm pelvis đến khớp HipYaw của chân trái/phải:

$$\vec{p}_{hip,L} = \begin{pmatrix} 0 \\ +d \\ -h_z \end{pmatrix}, \quad \vec{p}_{hip,R} = \begin{pmatrix} 0 \\ -d \\ -h_z \end{pmatrix}$$

Trong đó:
- d ≈ 35 mm (nửa khoảng cách hai hông)
- h_z ≈ 15 mm (offset dọc từ pelvis xuống trục hông)

---

## 3. Forward Kinematics

### 3.1 Ma trận quay cơ bản

Quay quanh trục X góc α:

$$R_x(\alpha) = \begin{pmatrix} 1 & 0 & 0 \\ 0 & \cos\alpha & -\sin\alpha \\ 0 & \sin\alpha & \cos\alpha \end{pmatrix}$$

Quay quanh trục Y góc β:

$$R_y(\beta) = \begin{pmatrix} \cos\beta & 0 & \sin\beta \\ 0 & 1 & 0 \\ -\sin\beta & 0 & \cos\beta \end{pmatrix}$$

Quay quanh trục Z góc γ:

$$R_z(\gamma) = \begin{pmatrix} \cos\gamma & -\sin\gamma & 0 \\ \sin\gamma & \cos\gamma & 0 \\ 0 & 0 & 1 \end{pmatrix}$$

### 3.2 Ma trận biến đổi thuần nhất 4×4

Ma trận biến đổi thuần nhất kết hợp quay và tịnh tiến:

$$T = \begin{pmatrix} R_{3\times3} & \vec{t}_{3\times1} \\ 0_{1\times3} & 1 \end{pmatrix}$$

### 3.3 Bảng Denavit-Hartenberg

Bảng DH cho chân trái (quy ước DH cải tiến):

| Link i | aᵢ | αᵢ | dᵢ | θᵢ |
|--------|-----|-----|-----|-----|
| 0→1 | 0 | 0 | 0 | θ₁ (HipYaw, quay Z) |
| 1→2 | 0 | π/2 | 0 | θ₂ (HipRoll, quay X) |
| 2→3 | 0 | π/2 | 0 | θ₃ (HipPitch, quay Y) |
| 3→4 | L₁ | 0 | 0 | θ₄ (KneePitch, quay Y) |
| 4→5 | L₂ | 0 | 0 | θ₅ (AnklePitch, quay Y) |
| 5→6 | 0 | −π/2 | 0 | θ₆ (AnkleRoll, quay X) |

### 3.4 Tính FK bằng tích ma trận

Vị trí bàn chân trong hệ pelvis:

$$T_{foot}^{pelvis} = T_{offset} \cdot T_1(\theta_1) \cdot T_2(\theta_2) \cdot T_3(\theta_3) \cdot T_4(\theta_4) \cdot T_5(\theta_5) \cdot T_6(\theta_6)$$

Triển khai từng bước:

**Bước 0** — Offset từ pelvis đến gốc chuỗi khớp:

$$T_{offset} = \begin{pmatrix} I_{3\times3} & \vec{p}_{hip} \\ 0 & 1 \end{pmatrix}$$

**Bước 1** — HipYaw (quay quanh Z):

$$T_1 = \begin{pmatrix} R_z(\theta_1) & \vec{0} \\ 0 & 1 \end{pmatrix}$$

**Bước 2** — HipRoll (quay quanh X):

$$T_2 = \begin{pmatrix} R_x(\theta_2) & \vec{0} \\ 0 & 1 \end{pmatrix}$$

**Bước 3** — HipPitch (quay quanh Y) + tịnh tiến xuống đùi:

$$T_3 = \begin{pmatrix} R_y(\theta_3) & \begin{pmatrix} 0 \\ 0 \\ -L_1 \end{pmatrix} \\ 0 & 1 \end{pmatrix}$$

**Bước 4** — KneePitch (quay quanh Y) + tịnh tiến xuống ống chân:

$$T_4 = \begin{pmatrix} R_y(\theta_4) & \begin{pmatrix} 0 \\ 0 \\ -L_2 \end{pmatrix} \\ 0 & 1 \end{pmatrix}$$

**Bước 5** — AnklePitch (quay quanh Y):

$$T_5 = \begin{pmatrix} R_y(\theta_5) & \vec{0} \\ 0 & 1 \end{pmatrix}$$

**Bước 6** — AnkleRoll (quay quanh X) + tịnh tiến xuống bàn chân:

$$T_6 = \begin{pmatrix} R_x(\theta_6) & \begin{pmatrix} 0 \\ 0 \\ -h_0 \end{pmatrix} \\ 0 & 1 \end{pmatrix}$$

### 3.5 Đơn giản hóa — Mặt phẳng Sagittal

Khi đi thẳng, θ₁ ≈ 0, θ₂ ≈ 0, θ₆ ≈ 0. Bài toán giảm về 3-DOF trong mặt phẳng XZ:

Vị trí ankle so với hip:

$$p_x = L_1 \sin\theta_3 + L_2 \sin(\theta_3 + \theta_4)$$

$$p_z = -L_1 \cos\theta_3 - L_2 \cos(\theta_3 + \theta_4)$$

---

## 4. Inverse Kinematics

### 4.1 Bài toán

**Cho:** Vị trí bàn chân mong muốn $\vec{p}_f = (p_x, p_y, p_z)$ trong hệ pelvis, và góc xoay chân mong muốn ψ.

**Tìm:** 6 góc khớp (θ₁, θ₂, θ₃, θ₄, θ₅, θ₆).

### 4.2 Giải θ₁ — HipYaw

$$\theta_1 = \psi$$

Khi đi thẳng, ψ = 0. Khi xoay, ψ là hướng bước.

### 4.3 Chuyển tọa độ về sau HipYaw

Xoay ngược θ₁ và trừ offset hip:

$$\vec{p}' = R_z(-\theta_1) \cdot (\vec{p}_f - \vec{p}_{hip})$$

$$p'_x = \cos\theta_1 \cdot (p_x - p_{hip,x}) + \sin\theta_1 \cdot (p_y - p_{hip,y})$$
$$p'_y = -\sin\theta_1 \cdot (p_x - p_{hip,x}) + \cos\theta_1 \cdot (p_y - p_{hip,y})$$
$$p'_z = p_z - p_{hip,z}$$

### 4.4 Giải θ₄ — KneePitch (Cosine Rule)

Khoảng cách từ HipPitch đến AnklePitch:

$$L = \sqrt{p'^2_x + p'^2_y + p'^2_z}$$

Áp dụng định lý cosine cho tam giác Hip-Knee-Ankle:

$$\cos\alpha_k = \frac{L_1^2 + L_2^2 - L^2}{2 \cdot L_1 \cdot L_2}$$

Góc gập gối:

$$\theta_4 = \pi - \arccos\left(\text{clamp}(\cos\alpha_k, \ -1, \ +1)\right)$$

**Điều kiện tồn tại nghiệm:**
- $|L_1 - L_2| \leq L \leq L_1 + L_2$
- Nếu $L > L_1 + L_2$: bàn chân ngoài tầm → clamp L
- Nếu $L < |L_1 - L_2|$: quá gần → clamp L

**Ràng buộc bent-knee:**

$$\theta_4 \geq \theta_{4,min} \quad (\theta_{4,min} \approx 15° \div 25°)$$

Nếu IK cho $\theta_4 < \theta_{4,min}$, buộc $\theta_4 = \theta_{4,min}$ và tính lại $p_z$:

$$L_{forced} = \sqrt{L_1^2 + L_2^2 - 2 L_1 L_2 \cos(\pi - \theta_{4,min})}$$

$$p'_z = -\sqrt{L_{forced}^2 - p'^2_x - p'^2_y}$$

### 4.5 Giải θ₃ — HipPitch

Góc từ hip đến ankle trong mặt phẳng sagittal:

$$\alpha = \text{atan2}(p'_x, \ -p'_z)$$

Góc từ đùi đến đường hip-ankle (cosine rule lần 2):

$$\beta = \arccos\left(\frac{L_1^2 + L^2 - L_2^2}{2 \cdot L_1 \cdot L}\right)$$

$$\theta_3 = \alpha - \beta$$

### 4.6 Giải θ₅ — AnklePitch

Điều kiện: bàn chân song song mặt đất (pitch = 0):

$$\theta_5 = -(\theta_3 + \theta_4)$$

Nếu muốn bàn chân nghiêng góc φ so với mặt đất:

$$\theta_5 = -(\theta_3 + \theta_4) + \varphi$$

### 4.7 Giải θ₂, θ₆ — HipRoll và AnkleRoll

Góc nghiêng ngang cần thiết:

$$\gamma = \text{atan2}(p'_y, \ -p'_z)$$

$$\theta_2 = \gamma$$

Bù lại để bàn chân nằm ngang:

$$\theta_6 = -\gamma$$

### 4.8 Tóm tắt IK hoàn chỉnh

Cho $\vec{p}_f = (p_x, p_y, p_z)$ và yaw ψ:

$$\boxed{\begin{aligned}
\theta_1 &= \psi \\[4pt]
\vec{p}' &= R_z(-\theta_1)(\vec{p}_f - \vec{p}_{hip}) \\[4pt]
L &= \|\vec{p}'\| \\[4pt]
\theta_4 &= \pi - \arccos\!\left(\frac{L_1^2 + L_2^2 - L^2}{2L_1 L_2}\right) \\[4pt]
\theta_3 &= \text{atan2}(p'_x, -p'_z) - \arccos\!\left(\frac{L_1^2 + L^2 - L_2^2}{2L_1 L}\right) \\[4pt]
\theta_5 &= -(\theta_3 + \theta_4) \\[4pt]
\theta_2 &= \text{atan2}(p'_y, -p'_z) \\[4pt]
\theta_6 &= -\theta_2
\end{aligned}}$$

### 4.9 Mirroring chân phải

Chân phải đối xứng gương qua mặt phẳng XZ. Khi áp dụng góc vào servo:

$$\theta_{1,R} = -\theta_1, \quad \theta_{2,R} = -\theta_2, \quad \theta_{6,R} = -\theta_6$$

$$\theta_{3,R} = \theta_3, \quad \theta_{4,R} = \theta_4, \quad \theta_{5,R} = \theta_5$$

Điều này tương ứng với `direction = -1` cho HipYaw, HipRoll, AnkleRoll ở chân phải.

### 4.10 Chuyển đổi góc robot → góc servo

Servo SG92R nhận góc 0°–180°, với 90° là vị trí giữa. Quy ước:

$$\theta_{servo} = 90° + \theta_{robot} \cdot \text{dir} + \text{offset}$$

Trong đó:
- $\theta_{robot}$ : góc từ IK (đơn vị degree, 0° = tư thế đứng)
- dir : +1 (chân trái) hoặc −1 (trục đảo ở chân phải)
- offset : sai số cơ khí được calibrate (degree)

---

## 5. Sinh quỹ đạo bước đi

### 5.1 Tham số bước

| Ký hiệu | Ý nghĩa |
|----------|---------|
| S | Chiều dài bước (step length, mm) |
| H | Chiều cao nhấc chân (step height, mm) |
| T_step | Thời gian 1 bước (s) |
| T = 2·T_step | Chu kỳ 1 stride (s) |
| z_c | Chiều cao trọng tâm so với mặt đất (mm) |
| W | Biên độ lắc ngang thân (body sway, mm) |
| ρ | Tỷ lệ double support (0 < ρ < 0.5) |

### 5.2 Phase timing

Một stride (2 bước) chia thành:

$$\phi(t) = \frac{t \mod T}{T} \in [0, 1)$$

Nửa đầu $\phi \in [0, 0.5)$: chân trái trụ, chân phải swing.
Nửa sau $\phi \in [0.5, 1)$: chân phải trụ, chân trái swing.

Trong mỗi nửa, pha cục bộ:

$$\phi_{local} = \frac{\phi \mod 0.5}{0.5} \in [0, 1)$$

Chia thành 3 giai đoạn:
- **Double support đầu**: $\phi_{local} \in [0, \rho)$
- **Single support (swing)**: $\phi_{local} \in [\rho, 1-\rho)$
- **Double support cuối**: $\phi_{local} \in [1-\rho, 1)$

Pha swing chuẩn hóa:

$$s = \frac{\phi_{local} - \rho}{1 - 2\rho} \in [0, 1]$$

### 5.3 Quỹ đạo chân swing — Trục X (tiến)

Dùng hàm **cycloid cải tiến** (modified cycloid) để đảm bảo vận tốc = 0 ở hai đầu:

$$x_{swing}(s) = S \left( s - \frac{1}{2\pi}\sin(2\pi s) \right) - \frac{S}{2}$$

**Tính chất:**
- $x_{swing}(0) = -S/2$ (vị trí sau)
- $x_{swing}(1) = +S/2$ (vị trí trước)
- $\dot{x}_{swing}(0) = \dot{x}_{swing}(1) = 0$ (vận tốc = 0 ở hai đầu)

Đạo hàm (vận tốc):

$$\dot{x}_{swing}(s) = \frac{S}{T_{swing}} \left(1 - \cos(2\pi s)\right)$$

### 5.4 Quỹ đạo chân swing — Trục Z (nâng)

Dùng **nửa sin**:

$$z_{swing}(s) = H \cdot \sin(\pi s)$$

**Tính chất:**
- $z_{swing}(0) = z_{swing}(1) = 0$ (chạm đất 2 đầu)
- $z_{swing}(0.5) = H$ (đỉnh ở giữa)
- $\dot{z}_{swing}(0) > 0$ (nhấc lên), $\dot{z}_{swing}(1) < 0$ (hạ xuống)

**Biến thể — Double peak** (tránh kéo chân trên mặt đất):

$$z_{swing}(s) = \begin{cases} H \cdot \sin(\pi \cdot 2s) & \text{nếu } s < 0.5 \\ H & \text{nếu } 0.35 \leq s \leq 0.65 \\ H \cdot \sin(\pi \cdot 2(1-s)) & \text{nếu } s > 0.5 \end{cases}$$

**Biến thể — Bézier bậc 3** (kiểm soát hình dạng tốt hơn):

4 điểm điều khiển trong mặt phẳng XZ:
- P₀ = (−S/2, 0)
- P₁ = (−S/4, κH) với κ ≈ 1.5 (overshoot để đỉnh tròn)
- P₂ = (+S/4, κH)
- P₃ = (+S/2, 0)

$$\vec{B}(s) = (1-s)^3 P_0 + 3(1-s)^2 s \, P_1 + 3(1-s) s^2 P_2 + s^3 P_3$$

### 5.5 Quỹ đạo chân trụ (support)

Chân trụ đứng yên trên mặt đất, nên trong hệ tọa độ mặt đất nó cố định. Trong hệ pelvis (di chuyển), chân trụ trượt ngược:

$$x_{support}(s) = \frac{S}{2} - S \cdot s$$

$$z_{support}(s) = 0$$

### 5.6 Quỹ đạo trọng tâm — Trục Y (lắc ngang)

Trọng tâm phải dịch sang chân trụ trước khi nhấc chân swing. Dùng sin:

$$y_{CoM}(\phi) = W \cdot \sin(2\pi\phi)$$

Quy ước: $\phi = 0$ bắt đầu với chân trái trụ → CoM dịch sang trái (Y dương).

### 5.7 Quỹ đạo trọng tâm — Trục X (tiến)

Trọng tâm tiến đều:

$$x_{CoM}(\phi) = v \cdot t = \frac{S}{T} \cdot t$$

### 5.8 Quỹ đạo trọng tâm — Trục Z (nhún)

Khi bước, trọng tâm hạ xuống nhẹ giữa pha double support và nâng lên giữa pha single support:

$$z_{CoM}(\phi) = z_c + \Delta z \cdot \cos(4\pi\phi)$$

Trong đó $\Delta z$ ≈ 2-5 mm — biên độ nhún nhỏ. Hệ số 4π vì có 2 chu kỳ nhún trong 1 stride.

### 5.9 Từ quỹ đạo CoM → vị trí bàn chân trong hệ pelvis

Vị trí bàn chân mong muốn (đầu vào IK) = vị trí tuyệt đối bàn chân − vị trí pelvis:

$$\vec{p}_{foot}^{pelvis} = \vec{p}_{foot}^{world} - \vec{p}_{CoM}^{world}$$

Cụ thể cho chân trái:

$$p_{fL,x} = x_{foot,L} - x_{CoM}$$
$$p_{fL,y} = +d - y_{CoM}$$
$$p_{fL,z} = z_{foot,L} - z_{CoM}$$

### 5.10 Góc thân trên

**TorsoRoll** — bù nghiêng do lắc CoM:

$$\theta_{T2} = -\arctan\!\left(\frac{y_{CoM}}{z_c}\right) \cdot k_{torso}$$

Với $k_{torso} \approx 0.3 \div 0.5$ — hệ số bù (không bù hết 100% vì thân trên nhẹ).

**TorsoYaw** — xoay theo hướng bước:

$$\theta_{T1} = -\Delta\psi \cdot k_{yaw}$$

Với $\Delta\psi$ là hiệu góc yaw giữa 2 chân, $k_{yaw} \approx 0.3$.

---

## 6. Zero Moment Point (ZMP)

### 6.1 Định nghĩa

ZMP là điểm trên mặt đất nơi tổng moment do lực quán tính và trọng lực bằng 0:

$$\vec{\tau}_{ZMP} = \vec{0}$$

### 6.2 Công thức ZMP

Cho robot với trọng tâm tại $(x_c, y_c, z_c)$ và gia tốc $(\ddot{x}_c, \ddot{y}_c, \ddot{z}_c)$:

$$x_{ZMP} = x_c - z_c \cdot \frac{\ddot{x}_c}{\ddot{z}_c + g}$$

$$y_{ZMP} = y_c - z_c \cdot \frac{\ddot{y}_c}{\ddot{z}_c + g}$$

### 6.3 Xấp xỉ ZMP từ IMU

Accelerometer đo gia tốc riêng (specific force) $\vec{a}_{IMU} = \vec{a} - \vec{g}$ trong hệ body. Chuyển sang hệ thế giới:

$$\vec{a}^{world} = R_{body}^{world} \cdot \vec{a}_{IMU}$$

Với $R_{body}^{world}$ tính từ estimated roll và pitch.

Xấp xỉ đơn giản (robot gần thẳng đứng):

$$x_{ZMP} \approx x_c - z_c \cdot \frac{a_x^{world}}{a_z^{world} + g}$$

$$y_{ZMP} \approx y_c - z_c \cdot \frac{a_y^{world}}{a_z^{world} + g}$$

### 6.4 Đa giác chống đỡ (Support Polygon)

**Pha single support** — 1 chân chạm đất:
- Support polygon = hình chữ nhật bàn chân
- Kích thước nhỏ (≈ 20×40 mm cho SG92R robot)

**Pha double support** — 2 chân chạm đất:
- Support polygon = hình bao lồi của 2 bàn chân
- Lớn hơn đáng kể

**Điều kiện ổn định:**

$$\vec{p}_{ZMP} \in \text{ConvexHull}(\text{support polygon})$$

### 6.5 Margin ổn định

Khoảng cách từ ZMP đến cạnh gần nhất của support polygon:

$$d_{margin} = \min_i \frac{|\vec{n}_i \cdot (\vec{p}_{ZMP} - \vec{v}_i)|}{\|\vec{n}_i\|}$$

Trong đó $\vec{n}_i$ là pháp tuyến trong (inward normal) của cạnh thứ i, $\vec{v}_i$ là đỉnh.

- $d_{margin} > 0$: ổn định
- $d_{margin} \leq 0$: mất cân bằng → emergency

---

## 7. Ước lượng trạng thái — Sensor Fusion

### 7.1 Bài toán

IMU cung cấp:
- Accelerometer: $\vec{a} = (a_x, a_y, a_z)$ — nhiễu cao, chính xác dài hạn
- Gyroscope: $\vec{\omega} = (\omega_x, \omega_y, \omega_z)$ — nhiễu thấp nhưng drift

Cần ước lượng roll (φ) và pitch (θ) chính xác.

### 7.2 Góc nghiêng từ accelerometer

Khi robot tĩnh hoặc chuyển động đều, gia tốc ≈ trọng lực:

$$\phi_{accel} = \text{atan2}(a_y, \ a_z)$$

$$\theta_{accel} = \text{atan2}\!\left(-a_x, \ \sqrt{a_y^2 + a_z^2}\right)$$

**Lưu ý:** IMU gắn ngược nên đã flip X, Z trong firmware. Các công thức trên áp dụng cho dữ liệu **sau khi flip**.

### 7.3 Complementary Filter

Kết hợp gyro (nhanh, drift) và accel (chậm, nhiễu):

$$\hat{\phi}(t) = \alpha \left[\hat{\phi}(t - \Delta t) + \omega_x \cdot \Delta t \right] + (1 - \alpha) \cdot \phi_{accel}(t)$$

$$\hat{\theta}(t) = \alpha \left[\hat{\theta}(t - \Delta t) + \omega_y \cdot \Delta t \right] + (1 - \alpha) \cdot \theta_{accel}(t)$$

Trong đó:

$$\alpha = \frac{\tau}{\tau + \Delta t}$$

- τ = hằng số thời gian (time constant), thường 0.5–2.0 s
- τ lớn → tin gyro hơn (phản ứng nhanh, nhưng drift nhiều)
- τ nhỏ → tin accel hơn (ổn định, nhưng nhiễu khi chuyển động)
- Gợi ý: **τ = 0.98 · Δt / (1 − 0.98) ≈ 1.0 s** cho Δt = 20 ms

### 7.4 Yaw từ magnetometer (tilt-compensated)

Magnetometer đo $\vec{m} = (m_x, m_y, m_z)$. Khi IMU nghiêng, cần bù tilt:

$$m'_x = m_x \cos\hat{\theta} + m_y \sin\hat{\phi}\sin\hat{\theta} + m_z \cos\hat{\phi}\sin\hat{\theta}$$

$$m'_y = m_y \cos\hat{\phi} - m_z \sin\hat{\phi}$$

$$\hat{\psi} = \text{atan2}(-m'_y, \ m'_x)$$

### 7.5 Kalman Filter (nâng cao)

Thay thế complementary filter khi cần chính xác hơn.

**Trạng thái:**

$$\vec{x} = \begin{pmatrix} \phi \\ \theta \\ b_{\omega_x} \\ b_{\omega_y} \end{pmatrix}$$

Trong đó $b_{\omega_x}, b_{\omega_y}$ là bias gyro (drift).

**Mô hình trạng thái (prediction):**

$$\vec{x}_{k|k-1} = \vec{x}_{k-1} + \begin{pmatrix} (\omega_x - b_{\omega_x}) \cdot \Delta t \\ (\omega_y - b_{\omega_y}) \cdot \Delta t \\ 0 \\ 0 \end{pmatrix}$$

$$P_{k|k-1} = F \cdot P_{k-1} \cdot F^T + Q$$

Với:

$$F = \begin{pmatrix} 1 & 0 & -\Delta t & 0 \\ 0 & 1 & 0 & -\Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{pmatrix}, \quad Q = \begin{pmatrix} q_\phi & 0 & 0 & 0 \\ 0 & q_\theta & 0 & 0 \\ 0 & 0 & q_b & 0 \\ 0 & 0 & 0 & q_b \end{pmatrix}$$

**Đo lường (update):**

$$\vec{z}_k = \begin{pmatrix} \phi_{accel} \\ \theta_{accel} \end{pmatrix}, \quad H = \begin{pmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{pmatrix}$$

$$K_k = P_{k|k-1} H^T (H P_{k|k-1} H^T + R)^{-1}$$

$$\vec{x}_k = \vec{x}_{k|k-1} + K_k (\vec{z}_k - H \vec{x}_{k|k-1})$$

$$P_k = (I - K_k H) P_{k|k-1}$$

Trong đó $R$ = noise covariance của accelerometer.

Giá trị khởi đầu đề xuất:
- $q_\phi = q_\theta = 0.001$
- $q_b = 0.003$
- $R = \text{diag}(0.03, \ 0.03)$

---

## 8. Bộ điều khiển ổn định

### 8.1 Phương trình Euler-Lagrange cho cân bằng

#### 8.1.1 Dẫn dắt từ cơ học Lagrange

Robot humanoid khi cân bằng xấp xỉ **con lắc ngược kép** (double inverted pendulum) trong 2 mặt phẳng: sagittal (pitch) và frontal (roll).

**Lagrangian** của hệ:

$$\mathcal{L} = T - V$$

Trong đó $T$ = động năng, $V$ = thế năng.

Cho con lắc ngược đơn (simplified) với khối lượng tập trung m tại chiều cao $z_c$, nghiêng góc nhỏ $\phi$ (roll) hoặc $\theta$ (pitch):

**Động năng:**

$$T = \frac{1}{2} m z_c^2 \dot{\phi}^2 + \frac{1}{2} m z_c^2 \dot{\theta}^2$$

**Thế năng** (lấy mốc tại chân):

$$V = m g z_c \cos\phi \cos\theta \approx m g z_c \left(1 - \frac{\phi^2}{2} - \frac{\theta^2}{2}\right)$$

(Xấp xỉ góc nhỏ: $\cos\alpha \approx 1 - \alpha^2/2$)

**Phương trình Euler-Lagrange:**

$$\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{q}_i} - \frac{\partial \mathcal{L}}{\partial q_i} = \tau_i + Q_i^{diss}$$

Với $q_1 = \phi$, $q_2 = \theta$, $Q_i^{diss} = -b_i \dot{q}_i$ (ma sát viscous):

**Cho roll:**

$$m z_c^2 \ddot{\phi} - m g z_c \phi = \tau_\phi - b_\phi \dot{\phi}$$

$$\ddot{\phi} = \frac{g}{z_c}\phi - \frac{b_\phi}{m z_c^2}\dot{\phi} + \frac{\tau_\phi}{m z_c^2}$$

**Cho pitch:**

$$\ddot{\theta} = \frac{g}{z_c}\theta - \frac{b_\theta}{m z_c^2}\dot{\theta} + \frac{\tau_\theta}{m z_c^2}$$

#### 8.1.2 Moment điều khiển từ khớp

Moment $\tau_\phi$ (roll) được tạo bởi 3 khớp:

$$\tau_\phi = \tau_{ankle,roll} + \tau_{hip,roll} + \tau_{torso,roll}$$

Mỗi khớp tạo moment tỷ lệ với thay đổi góc và cánh tay đòn:

$$\tau_{ankle,roll} = m g \cdot h_0 \cdot \Delta\theta_{ankle,roll}$$

$$\tau_{hip,roll} = m g \cdot (z_c - h_0) \cdot \Delta\theta_{hip,roll}$$

$$\tau_{torso,roll} = m_{upper} g \cdot l_{torso} \cdot \Delta\theta_{torso,roll}$$

Tương tự cho pitch:

$$\tau_\theta = m g \cdot h_0 \cdot \Delta\theta_{ankle,pitch} + m g \cdot (z_c - h_0) \cdot \Delta\theta_{hip,pitch}$$

#### 8.1.3 Phương trình tổng quát (2 trục, 5 đầu vào)

$$\begin{pmatrix} \ddot{\phi} \\ \ddot{\theta} \end{pmatrix} = \underbrace{\begin{pmatrix} \omega_n^2 & 0 \\ 0 & \omega_n^2 \end{pmatrix}}_{\text{bất ổn định}} \begin{pmatrix} \phi \\ \theta \end{pmatrix} - \underbrace{\begin{pmatrix} \beta_\phi & 0 \\ 0 & \beta_\theta \end{pmatrix}}_{\text{damping}} \begin{pmatrix} \dot{\phi} \\ \dot{\theta} \end{pmatrix} + \underbrace{\begin{pmatrix} b_{11} & b_{12} & b_{13} & 0 & 0 \\ 0 & 0 & 0 & b_{24} & b_{25} \end{pmatrix}}_{\text{input coupling}} \vec{u}$$

Trong đó:

$$\omega_n^2 = \frac{g}{z_c}, \quad \beta_\phi = \frac{b_\phi}{m z_c^2}, \quad \beta_\theta = \frac{b_\theta}{m z_c^2}$$

$$b_{11} = \frac{g \cdot h_0}{z_c^2}, \quad b_{12} = \frac{g \cdot (z_c - h_0)}{z_c^2}, \quad b_{13} = \frac{g}{z_c} \cdot \frac{m_{upper}}{m} \cdot \frac{l_{torso}}{z_c}$$

$$b_{24} = b_{11}, \quad b_{25} = b_{12}$$

**Thay giá trị PNOID** (ước lượng):
- $z_c = 0.09$ m, $h_0 = 0.02$ m, $m = 0.5$ kg, $m_{upper}/m = 0.3$, $l_{torso} = 0.03$ m
- $\omega_n^2 = 9.81/0.09 = 109$ rad²/s²
- $b_{11} = 9.81 \times 0.02 / 0.09^2 = 24.2$
- $b_{12} = 9.81 \times 0.07 / 0.09^2 = 84.8$
- $b_{13} = 109 \times 0.3 \times 0.03/0.09 = 10.9$

→ **Hip có ảnh hưởng lớn nhất** ($b_{12}$), nhưng ankle phản ứng nhanh nhất.

#### 8.1.4 Coupling giữa Roll và Pitch

Phương trình trên giả sử roll-pitch độc lập. Thực tế có coupling bậc 2:

$$\ddot{\phi} = \omega_n^2 \phi + \underbrace{\dot{\theta}\dot{\phi}\frac{\sin 2\phi}{2}}_{\text{gyroscopic}} + ... $$

Với góc nhỏ ($\phi, \theta < 15°$), coupling ≈ 0. Nhưng khi lắc mạnh (walking), coupling tăng. MPC xử lý tốt hơn LQR vì cập nhật prediction model mỗi bước.

---

### 8.2 Mô hình không gian trạng thái (State-Space Model)

#### 8.2.1 Định nghĩa

Từ phương trình Euler-Lagrange (Mục 8.1.3), chuyển sang dạng state-space.

**Trạng thái** (state vector):

$$\vec{x} = \begin{pmatrix} \phi \\ \theta \\ \dot{\phi} \\ \dot{\theta} \\ \xi_\phi \\ \xi_\theta \end{pmatrix} \in \mathbb{R}^6$$

Trong đó:
- $\phi$ = roll angle (nghiêng ngang)
- $\theta$ = pitch angle (nghiêng trước/sau)
- $\dot{\phi}, \dot{\theta}$ = tốc độ góc (từ gyro)
- $\xi_\phi, \xi_\theta$ = trạng thái tích phân của sai số (cho LQI, xem 8.5)

**Đầu vào** (input vector):

$$\vec{u} = \begin{pmatrix} \Delta\theta_{ankle,roll} \\ \Delta\theta_{hip,roll} \\ \Delta\theta_{torso,roll} \\ \Delta\theta_{ankle,pitch} \\ \Delta\theta_{hip,pitch} \end{pmatrix} \in \mathbb{R}^5$$

5 đầu vào = 5 góc bù tại các khớp chính.

**Đầu ra** (output vector):

$$\vec{y} = \begin{pmatrix} \phi \\ \theta \end{pmatrix} \in \mathbb{R}^2$$

#### 8.2.2 Tuyến tính hóa quanh điểm làm việc

Robot đi bộ ở trạng thái "gần thẳng đứng" (bent-knee nhưng thân gần thẳng). Tuyến tính hóa quanh $\vec{x}_0 = \vec{0}$:

**Mô hình liên tục:**

$$\dot{\vec{x}} = A_c \vec{x} + B_c \vec{u}$$

$$\vec{y} = C \vec{x}$$

**Ma trận A** (dynamics): Từ LIPM, tuyến tính hóa trọng lực:

$$A_c = \begin{pmatrix} 0 & 0 & 1 & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 & 0 & 0 \\ \omega_n^2 & 0 & -b_\phi & 0 & 0 & 0 \\ 0 & \omega_n^2 & 0 & -b_\theta & 0 & 0 \\ 1 & 0 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 & 0 \end{pmatrix}$$

Trong đó:
- $\omega_n^2 = g / z_c$ — hệ số bất ổn định từ LIPM (con lắc ngược)
- $b_\phi, b_\theta$ — hệ số ma sát/damping (ước lượng thực nghiệm, ~0.5–2.0 rad/s)
- Hai hàng cuối là integrator: $\dot{\xi}_\phi = \phi, \ \dot{\xi}_\theta = \theta$

**Ma trận B** (input coupling): Mô hình hóa ảnh hưởng của mỗi khớp lên roll/pitch:

$$B_c = \begin{pmatrix} 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 \\ b_{11} & b_{12} & b_{13} & 0 & 0 \\ 0 & 0 & 0 & b_{24} & b_{25} \\ 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 \end{pmatrix}$$

Trong đó $b_{ij}$ phụ thuộc hình học robot:

$$b_{11} = \frac{g}{z_c} \cdot \frac{l_{ankle}}{z_c} \approx \frac{g \cdot h_0}{z_c^2}$$

$$b_{12} = \frac{g}{z_c} \cdot \frac{l_{hip}}{z_c} \approx \frac{g \cdot (z_c - h_0)}{z_c^2}$$

$$b_{13} = \frac{g}{z_c} \cdot \frac{m_{upper}}{m_{total}} \cdot \frac{l_{torso}}{z_c}$$

$$b_{24} \approx b_{11}, \quad b_{25} \approx b_{12} \quad \text{(tương tự cho pitch)}$$

**Ý nghĩa vật lý:** $b_{11}$ lớn → ankle roll ảnh hưởng mạnh đến roll. $b_{12}$ nhỏ hơn nhưng hip roll tác động "sâu" hơn (gần trọng tâm).

**Ma trận C** (output):

$$C = \begin{pmatrix} 1 & 0 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 & 0 \end{pmatrix}$$

#### 8.2.3 Rời rạc hóa (Zero-Order Hold)

Với $\Delta t$ = 0.02s (50Hz):

$$A = e^{A_c \Delta t} \approx I + A_c \Delta t + \frac{(A_c \Delta t)^2}{2}$$

$$B = \left(\int_0^{\Delta t} e^{A_c \tau} d\tau \right) B_c \approx (I \cdot \Delta t + \frac{A_c \Delta t^2}{2}) B_c$$

Xấp xỉ bậc 1 (đủ chính xác khi $\|\omega_n \Delta t\| \ll 1$, kiểm tra: $10.4 \times 0.02 = 0.208$ — chấp nhận được):

$$A \approx I + A_c \Delta t$$

$$B \approx B_c \Delta t$$

Mô hình rời rạc:

$$\boxed{\vec{x}_{k+1} = A \vec{x}_k + B \vec{u}_k}$$

$$\vec{y}_k = C \vec{x}_k$$

---

### 8.3 Feedforward từ động lực học

#### 8.3.1 Nguyên lý

Quỹ đạo bước (Mục 5) đã biết trước → tính gia tốc mong muốn → moment cần thiết → góc bù **trước khi sai số xảy ra**.

Feedforward không chờ sai số — nó "đoán trước". Feedback (LQR/MPC) chỉ sửa phần sai lệch còn lại.

$$\vec{u}_{total} = \underbrace{\vec{u}_{ff}}_{\text{feedforward}} + \underbrace{\vec{u}_{fb}}_{\text{feedback (LQR/MPC)}}$$

#### 8.3.2 Tính moment feedforward

Từ LIPM, gia tốc mong muốn của trọng tâm:

$$\ddot{x}_{ref} = \frac{g}{z_c}(x_{c,ref} - x_{ZMP,ref})$$

$$\ddot{y}_{ref} = \frac{g}{z_c}(y_{c,ref} - y_{ZMP,ref})$$

Moment cần thiết:

$$\tau_{ff,pitch} = m \cdot g \cdot (x_c - x_{ankle}) + m \cdot z_c \cdot \ddot{x}_{ref}$$

$$\tau_{ff,roll} = m \cdot g \cdot (y_c - y_{ankle}) + m \cdot z_c \cdot \ddot{y}_{ref}$$

#### 8.3.3 Chuyển moment → góc bù (inverse dynamics)

Từ quan hệ moment-khớp (Mục 8.1.2):

$$\tau_\phi = B_{moment} \cdot \vec{u}_{ff}$$

Trong đó $B_{moment} \in \mathbb{R}^{2 \times 5}$ chứa cánh tay đòn × khối lượng × g.

Bài toán under-determined (2 phương trình, 5 ẩn). Nghiệm minimum-norm:

$$\vec{u}_{ff} = B_{moment}^T (B_{moment} B_{moment}^T)^{-1} \begin{pmatrix} \tau_{ff,roll} \\ \tau_{ff,pitch} \end{pmatrix}$$

Khai triển với ma trận 2×5, nghịch đảo 2×2 giải tích:

$$\begin{pmatrix} a & b \\ c & d \end{pmatrix}^{-1} = \frac{1}{ad - bc} \begin{pmatrix} d & -b \\ -c & a \end{pmatrix}$$

#### 8.3.4 Đạo hàm quỹ đạo cho gia tốc

Gia tốc trọng tâm tính từ đạo hàm bậc 2 quỹ đạo CoM (Mục 5.6–5.8):

$$\ddot{y}_{CoM} = -W \cdot (2\pi/T)^2 \cdot \sin(2\pi\phi)$$

$$\ddot{z}_{CoM} = -\Delta z \cdot (4\pi/T)^2 \cdot \cos(4\pi\phi)$$

Đạo hàm cycloid (Mục 5.3):

$$\ddot{x}_{swing} = \frac{2\pi S}{T_{swing}^2} \sin(2\pi s)$$

Tất cả đã biết trước (từ gait parameters) → tính offline hoặc realtime rẻ (chỉ sin/cos).

---

### 8.4 LQR — Linear Quadratic Regulator

#### 8.4.1 Bài toán tối ưu

Tìm $\vec{u}_k$ sao cho **tối thiểu hóa** hàm chi phí:

$$\boxed{J = \sum_{k=0}^{\infty} \left( \vec{x}_k^T Q \vec{x}_k + \vec{u}_k^T R \vec{u}_k \right)}$$

Trong đó:
- $Q \in \mathbb{R}^{6 \times 6}$ — ma trận trọng số trạng thái (semipositive definite, $Q \geq 0$)
- $R \in \mathbb{R}^{5 \times 5}$ — ma trận trọng số đầu vào (positive definite, $R > 0$)

**Ý nghĩa:**
- Q lớn → phạt nặng sai lệch trạng thái → robot bám sát target → nhưng dùng nhiều năng lượng
- R lớn → phạt nặng đầu vào → servo chuyển động nhẹ nhàng → nhưng tracking chậm hơn
- **Tradeoff Q vs R** là bản chất của LQR

#### 8.4.2 Chọn ma trận Q

Ma trận Q thường chọn dạng đường chéo:

$$Q = \text{diag}(q_\phi, \ q_\theta, \ q_{\dot\phi}, \ q_{\dot\theta}, \ q_{\xi_\phi}, \ q_{\xi_\theta})$$

| Phần tử | Ý nghĩa | Giá trị đề xuất | Lý do |
|---------|---------|-----------------|-------|
| $q_\phi$ | Phạt nghiêng roll | 500 | Roll nguy hiểm → ngã ngang |
| $q_\theta$ | Phạt nghiêng pitch | 300 | Pitch nguy hiểm nhưng có 2 chân đỡ trước/sau |
| $q_{\dot\phi}$ | Phạt tốc độ roll | 10 | Damping, chống dao động |
| $q_{\dot\theta}$ | Phạt tốc độ pitch | 10 | Damping |
| $q_{\xi_\phi}$ | Phạt tích phân roll | 50 | Triệt tiêu steady-state error roll |
| $q_{\xi_\theta}$ | Phạt tích phân pitch | 30 | Triệt tiêu steady-state error pitch |

**Tại sao $q_\phi > q_\theta$?** Robot chỉ rộng ~70mm (2d) nhưng dài hơn (bàn chân). Support polygon hẹp theo Y → dễ ngã ngang hơn.

#### 8.4.3 Chọn ma trận R

$$R = \text{diag}(r_{ankle,roll}, \ r_{hip,roll}, \ r_{torso,roll}, \ r_{ankle,pitch}, \ r_{hip,pitch})$$

| Phần tử | Giá trị đề xuất | Lý do |
|---------|-----------------|-------|
| $r_{ankle,roll}$ | 1 | Ankle phản ứng nhanh, ưu tiên dùng |
| $r_{hip,roll}$ | 3 | Hip chậm hơn, ít dùng |
| $r_{torso,roll}$ | 5 | Torso chỉ bù nhẹ, tránh giật thân trên |
| $r_{ankle,pitch}$ | 1 | Tương tự ankle roll |
| $r_{hip,pitch}$ | 3 | Tương tự hip roll |

**r nhỏ → LQR dùng khớp đó nhiều hơn.** Ankle r nhỏ nhất vì phản ứng nhanh nhất (gần mặt đất, moment nhỏ).

#### 8.4.4 Phương trình Riccati rời rạc (DARE)

Nghiệm tối ưu $\vec{u}_k = -K \vec{x}_k$ với K tìm bằng giải DARE:

$$\boxed{P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q}$$

Đây là phương trình **đại số Riccati rời rạc** (Discrete Algebraic Riccati Equation). Giải cho P (ma trận đối xứng xác định dương), sau đó:

$$\boxed{K = (R + B^T P B)^{-1} B^T P A}$$

**K ∈ ℝ^(5×6)** — mỗi hàng của K cho biết mỗi đầu vào (khớp) phản ứng với mỗi trạng thái bao nhiêu.

#### 8.4.5 Giải DARE — Phương pháp lặp

Trên STM32 không có thư viện DARE sẵn. Có 2 cách:

**Cách 1: Giải offline trên PC, nạp K vào firmware**
- Dùng MATLAB: `[K,P] = dlqr(A,B,Q,R)`
- Dùng Python: `from scipy.linalg import solve_discrete_are`
- Nạp K (5×6 = 30 hằng số) vào firmware

**Cách 2: Giải bằng lặp trên STM32** (nếu cần thay đổi online)

Khởi tạo: $P_0 = Q$

Lặp $n = 0, 1, 2, ...$:

$$K_n = (R + B^T P_n B)^{-1} B^T P_n A$$

$$P_{n+1} = A^T P_n A - A^T P_n B \cdot K_n + Q$$

Hội tụ khi $\|P_{n+1} - P_n\|_F < \epsilon$ (thường 20–50 lần lặp).

**Chi phí tính toán:** Mỗi lần lặp cần nhân ma trận 6×6 và nghịch đảo 5×5.
- Nhân 6×6: ~216 phép nhân
- Nghịch đảo 5×5: ~125 phép nhân (Gauss-Jordan)
- Tổng ~50 lặp × ~500 phép = ~25000 phép nhân → STM32H743 ở 480MHz xong trong <1ms

#### 8.4.6 Luật điều khiển LQR

$$\boxed{\vec{u}_k = -K \vec{x}_k}$$

Triển khai:

$$\begin{pmatrix} \Delta\theta_{ankle,roll} \\ \Delta\theta_{hip,roll} \\ \Delta\theta_{torso,roll} \\ \Delta\theta_{ankle,pitch} \\ \Delta\theta_{hip,pitch} \end{pmatrix}_k = -\begin{pmatrix} K_{11} & K_{12} & K_{13} & K_{14} & K_{15} & K_{16} \\ K_{21} & K_{22} & K_{23} & K_{24} & K_{25} & K_{26} \\ K_{31} & K_{32} & K_{33} & K_{34} & K_{35} & K_{36} \\ K_{41} & K_{42} & K_{43} & K_{44} & K_{45} & K_{46} \\ K_{51} & K_{52} & K_{53} & K_{54} & K_{55} & K_{56} \end{pmatrix} \begin{pmatrix} \phi_k \\ \theta_k \\ \dot\phi_k \\ \dot\theta_k \\ \xi_{\phi,k} \\ \xi_{\theta,k} \end{pmatrix}$$

**Ý nghĩa từng cột K:**
- Cột 1 ($K_{i1}$): phản ứng của khớp i khi có sai lệch roll → **tương đương Kp_roll cho mỗi khớp**
- Cột 3 ($K_{i3}$): phản ứng khi có tốc độ roll → **tương đương Kd_roll**
- Cột 5 ($K_{i5}$): phản ứng khi có tích phân roll → **tương đương Ki_roll**
- LQR tự động **phân bổ** correction giữa ankle/hip/torso **tối ưu** thay vì tỷ lệ cố định

#### 8.4.7 Tính chất LQR

**Định lý ổn định:** Nếu (A, B) khả điều khiển (controllable) và (A, Q^½) khả quan sát (observable), thì:
1. DARE có nghiệm duy nhất P > 0
2. Hệ kín $\vec{x}_{k+1} = (A - BK)\vec{x}_k$ ổn định tiệm cận
3. Mọi trị riêng của $(A - BK)$ nằm trong đường tròn đơn vị $|\lambda_i| < 1$

**Kiểm tra khả điều khiển:**

$$\mathcal{C} = \begin{pmatrix} B & AB & A^2B & \cdots & A^{n-1}B \end{pmatrix}$$

Hệ khả điều khiển khi $\text{rank}(\mathcal{C}) = n = 6$.

**Margin ổn định:** LQR đảm bảo:
- Gain margin ≥ 6 dB (tức gain có thể tăng gấp 2 mà vẫn ổn định)
- Phase margin ≥ 60°
- Rất robust so với PID (phase margin thường chỉ 30–45°)

---

### 8.5 LQI — LQR with Integral Action

#### 8.5.1 Tại sao cần thêm Integral?

LQR cơ bản (trạng thái 4: φ, θ, dφ, dθ) triệt tiêu sai số khi mô hình chính xác. Nhưng:
- Khung robot lệch tâm → bias moment không có trong mô hình
- SG92R sai số ~5° → góc thực ≠ góc command
- Trọng tâm thay đổi theo gait phase → disturb liên tục

Thêm **trạng thái tích phân** để bù disturb không mô hình được.

#### 8.5.2 Augmented state

Trạng thái mở rộng (đã bao gồm trong Mục 8.3.1):

$$\vec{x}_{aug} = \begin{pmatrix} \vec{x}_{orig} \\ \vec{\xi} \end{pmatrix}, \quad \dot{\xi}_i = y_i - y_{ref,i}$$

Rời rạc:

$$\xi_{\phi,k+1} = \xi_{\phi,k} + (\phi_k - \phi_{ref,k}) \cdot \Delta t$$

$$\xi_{\theta,k+1} = \xi_{\theta,k} + (\theta_k - \theta_{ref,k}) \cdot \Delta t$$

Ma trận augmented:

$$A_{aug} = \begin{pmatrix} A_{4\times4} & 0_{4\times2} \\ C \cdot \Delta t & I_{2\times2} \end{pmatrix}, \quad B_{aug} = \begin{pmatrix} B_{4\times5} \\ 0_{2\times5} \end{pmatrix}$$

Giải DARE với $(A_{aug}, B_{aug}, Q_{aug}, R)$ → gain $K_{aug} \in \mathbb{R}^{5 \times 6}$.

#### 8.5.3 Anti-windup cho LQI

Tương tự PID, tích phân cần giới hạn:

$$|\xi_{\phi,k}| \leq \xi_{max}, \quad |\xi_{\theta,k}| \leq \xi_{max}$$

Với $\xi_{max}$ chọn sao cho thành phần integral không vượt quá ~30% tổng correction:

$$\xi_{max} \approx \frac{0.3 \cdot u_{max}}{\|K_{aug}(:, 5:6)\|_\infty}$$

Khi bão hòa, dừng tích phân (conditional integration):

$$\xi_{k+1} = \begin{cases} \xi_k & \text{nếu } |\xi_k| = \xi_{max} \text{ và } \text{sign}(e_k) = \text{sign}(\xi_k) \\ \xi_k + e_k \Delta t & \text{ngược lại} \end{cases}$$

---

### 8.6 Phân tích ổn định — Lyapunov

#### 8.6.1 Hàm Lyapunov cho hệ kín LQR

Hệ kín sau khi áp dụng LQR:

$$\vec{x}_{k+1} = (A - BK)\vec{x}_k \triangleq A_{cl}\vec{x}_k$$

**Định lý Lyapunov rời rạc:** Hệ ổn định tiệm cận nếu tồn tại $V(\vec{x}) > 0$ sao cho $\Delta V < 0$.

Chọn hàm Lyapunov bậc 2 (candidate):

$$V(\vec{x}_k) = \vec{x}_k^T P \vec{x}_k$$

Với P là nghiệm DARE (ma trận đối xứng xác định dương).

**Chứng minh $\Delta V < 0$:**

$$\Delta V = V(\vec{x}_{k+1}) - V(\vec{x}_k)$$

$$= \vec{x}_{k+1}^T P \vec{x}_{k+1} - \vec{x}_k^T P \vec{x}_k$$

$$= \vec{x}_k^T A_{cl}^T P A_{cl} \vec{x}_k - \vec{x}_k^T P \vec{x}_k$$

$$= \vec{x}_k^T (A_{cl}^T P A_{cl} - P) \vec{x}_k$$

Từ DARE, có thể chứng minh:

$$A_{cl}^T P A_{cl} - P = -(Q + K^T R K)$$

Vì $Q \geq 0$ và $R > 0$ → $(Q + K^T R K) > 0$ → $\Delta V < 0$ ∀ $\vec{x} \neq 0$. ∎

#### 8.6.2 Ý nghĩa vật lý

$V(\vec{x}) = \vec{x}^T P \vec{x}$ có thể hiểu là **năng lượng tổng quát** của hệ:
- Chứa động năng ($\dot\phi^2, \dot\theta^2$)
- Chứa thế năng ($\phi^2, \theta^2$ — lệch khỏi cân bằng)
- Cross-term ($\phi \dot\phi$, ...) — coupling giữa vị trí và vận tốc

$\Delta V < 0$ nghĩa là **năng lượng giảm mỗi bước** → hệ trở về cân bằng.

Tốc độ hội tụ phụ thuộc trị riêng $\lambda_i$ của $A_{cl}$:

$$\|\vec{x}_k\| \leq \|\vec{x}_0\| \cdot |\lambda_{max}|^k$$

$|\lambda_{max}|$ càng nhỏ → hội tụ càng nhanh.

#### 8.6.3 Vùng hấp dẫn (Region of Attraction)

Lý thuyết LQR đảm bảo ổn định **toàn cục** cho mô hình tuyến tính. Nhưng robot thực **phi tuyến** → chỉ ổn định **cục bộ** trong vùng xấp xỉ tuyến tính hóa còn đúng.

Vùng hấp dẫn ước lượng: tập $\mathcal{E}$ sao cho xấp xỉ tuyến tính sai < 10%:

$$\mathcal{E} = \{ \vec{x} : |\phi| < \phi_{max}, |\theta| < \theta_{max} \}$$

Với xấp xỉ $\sin\alpha \approx \alpha$ (sai < 5% khi $|\alpha| < 18°$):

$$\phi_{max} \approx \theta_{max} \approx 18° \approx 0.31 \text{ rad}$$

Ngoài vùng này, cần controller phi tuyến hoặc emergency stance.

#### 8.6.4 Robustness Margins — Chứng minh

**Gain margin ≥ 1/2 đến ∞ (tức ≥ 6 dB):**

Xét loop transfer function: $L(z) = K(zI - A)^{-1}B$

Từ DARE, return difference inequality:

$$[I + L(e^{j\omega})]^H R [I + L(e^{j\omega})] \geq R \quad \forall \omega$$

Suy ra:

$$\sigma_{min}[I + L(e^{j\omega})] \geq 1$$

Đây tương đương:
- **Gain margin:** hệ ổn định khi gain nhân với $\alpha \in [1/2, +\infty)$
- **Phase margin:** $\geq 60°$
- Rất robust — so với PID thường chỉ có phase margin 30–45°

**Ý nghĩa thực tế:** Ngay cả khi mô hình sai 50% (gain sai gấp 2), LQR vẫn ổn định. Với SG92R sai số ~10%, đây là margin rất thoải mái.

#### 8.6.5 Ổn định MPC — Terminal Cost

MPC không tự đảm bảo ổn định. Cần thêm điều kiện:

**Terminal cost** $Q_f = P$ (từ DARE):

$$J_{terminal} = \vec{x}_{k+N_p}^T P \vec{x}_{k+N_p}$$

**Định lý (Mayne et al. 2000):** Nếu:
1. $Q_f = P$ (DARE solution)
2. Terminal constraint: $\vec{x}_{k+N_p} \in \mathcal{X}_f$ (terminal set — tập mà LQR giữ được)
3. $\mathcal{X}_f$ là positive invariant dưới LQR: $A_{cl}\vec{x} \in \mathcal{X}_f \ \forall \vec{x} \in \mathcal{X}_f$

Thì MPC **ổn định tiệm cận** và $V(\vec{x}_k) = J^*(\vec{x}_k)$ là hàm Lyapunov.

$\mathcal{X}_f$ tính bằng maximum invariant ellipsoid:

$$\mathcal{X}_f = \{\vec{x} : \vec{x}^T P \vec{x} \leq \gamma \}$$

Với $\gamma$ lớn nhất sao cho ∀ $\vec{x} \in \mathcal{X}_f$: constraints được thỏa.

---

### 8.7 MPC — Model Predictive Control

#### 8.7.1 Tại sao MPC?

LQR tối ưu nhưng **không xử lý được ràng buộc** (constraints):
- Giới hạn khớp: $\theta_{min} \leq \theta_i \leq \theta_{max}$
- Giới hạn tốc độ servo: $|\dot{\theta}_i| \leq \dot{\theta}_{max}$ (SG92R: ~600°/s)
- ZMP phải trong support polygon: $\vec{p}_{ZMP} \in \mathcal{S}$
- Correction tối đa: $|\Delta\theta_i| \leq \Delta\theta_{max}$

MPC giải **bài toán tối ưu có ràng buộc** ở mỗi bước.

#### 8.7.2 Bài toán tối ưu MPC

Tại mỗi bước k, cho trạng thái hiện tại $\vec{x}_k$, giải:

$$\boxed{\min_{\vec{u}_k, ..., \vec{u}_{k+N_p-1}} J = \sum_{j=0}^{N_p-1} \left( \vec{e}_{k+j}^T Q \vec{e}_{k+j} + \vec{u}_{k+j}^T R \vec{u}_{k+j} + \Delta\vec{u}_{k+j}^T S \Delta\vec{u}_{k+j} \right) + \vec{e}_{k+N_p}^T Q_f \vec{e}_{k+N_p}}$$

Với các ràng buộc:

$$\vec{x}_{k+j+1} = A \vec{x}_{k+j} + B \vec{u}_{k+j} \quad \forall j = 0, ..., N_p - 1$$

$$\vec{u}_{min} \leq \vec{u}_{k+j} \leq \vec{u}_{max} \quad \forall j$$

$$|\vec{u}_{k+j} - \vec{u}_{k+j-1}| \leq \Delta\vec{u}_{max} \quad \forall j$$

$$\vec{x}_{min} \leq \vec{x}_{k+j} \leq \vec{x}_{max} \quad \forall j$$

Trong đó:
- $N_p$ = **prediction horizon** (tầm nhìn), thường 5–15 bước
- $\vec{e}_{k+j} = \vec{x}_{k+j} - \vec{x}_{ref,k+j}$ — sai số tracking
- $\Delta\vec{u}_{k+j} = \vec{u}_{k+j} - \vec{u}_{k+j-1}$ — thay đổi đầu vào (phạt để servo chuyển mượt)
- $S$ = ma trận phạt thay đổi input (smooth factor)
- $Q_f$ = terminal cost (thường = P từ DARE, đảm bảo ổn định)

**Nguyên tắc Receding Horizon:**
1. Giải bài toán → được chuỗi $\vec{u}_k^*, ..., \vec{u}_{k+N_p-1}^*$
2. **Chỉ áp dụng** $\vec{u}_k^*$ (bước đầu tiên)
3. Bước tiếp: đo $\vec{x}_{k+1}$ mới, giải lại

#### 8.7.3 Chuyển về Quadratic Programming (QP)

Khai triển prediction model:

$$\vec{x}_{k+1} = A\vec{x}_k + B\vec{u}_k$$
$$\vec{x}_{k+2} = A^2\vec{x}_k + AB\vec{u}_k + B\vec{u}_{k+1}$$
$$\vdots$$
$$\vec{x}_{k+j} = A^j\vec{x}_k + \sum_{i=0}^{j-1} A^{j-1-i} B \vec{u}_{k+i}$$

Dạng ma trận gọn:

$$\vec{X} = \mathcal{A} \vec{x}_k + \mathcal{B} \vec{U}$$

Trong đó:

$$\vec{X} = \begin{pmatrix} \vec{x}_{k+1} \\ \vec{x}_{k+2} \\ \vdots \\ \vec{x}_{k+N_p} \end{pmatrix}, \quad \vec{U} = \begin{pmatrix} \vec{u}_k \\ \vec{u}_{k+1} \\ \vdots \\ \vec{u}_{k+N_p-1} \end{pmatrix}$$

$$\mathcal{A} = \begin{pmatrix} A \\ A^2 \\ \vdots \\ A^{N_p} \end{pmatrix}, \quad \mathcal{B} = \begin{pmatrix} B & 0 & \cdots & 0 \\ AB & B & \cdots & 0 \\ \vdots & & \ddots & \vdots \\ A^{N_p-1}B & A^{N_p-2}B & \cdots & B \end{pmatrix}$$

Thay vào hàm chi phí:

$$J = \vec{U}^T \underbrace{(\mathcal{B}^T \bar{Q} \mathcal{B} + \bar{R} + \bar{S})}_{H} \vec{U} + 2 \underbrace{(\mathcal{A}\vec{x}_k - \vec{X}_{ref})^T \bar{Q} \mathcal{B}}_{f^T} \vec{U} + \text{const}$$

Với $\bar{Q} = \text{diag}(Q, Q, ..., Q_f)$, $\bar{R} = \text{diag}(R, R, ..., R)$.

Ma trận $\bar{S}$ cho thành phần $\Delta u$:

$$\bar{S} = T^T \text{diag}(S, ..., S) T$$

Trong đó T là ma trận sai phân:

$$T = \begin{pmatrix} I & 0 & 0 & \cdots \\ -I & I & 0 & \cdots \\ 0 & -I & I & \cdots \\ \vdots & & \ddots & \ddots \end{pmatrix}$$

**Bài toán QP chuẩn:**

$$\boxed{\min_{\vec{U}} \frac{1}{2} \vec{U}^T H \vec{U} + f^T \vec{U}}$$

$$\text{s.t.} \quad G \vec{U} \leq h$$

Trong đó $G, h$ chứa tất cả ràng buộc bất đẳng thức.

#### 8.7.4 Ràng buộc cụ thể

**Giới hạn input:**

$$G_u = \begin{pmatrix} I_{5N_p} \\ -I_{5N_p} \end{pmatrix}, \quad h_u = \begin{pmatrix} \vec{u}_{max} \otimes \vec{1}_{N_p} \\ -\vec{u}_{min} \otimes \vec{1}_{N_p} \end{pmatrix}$$

**Giới hạn rate (ΔU):**

$$G_{\Delta u} = \begin{pmatrix} T \\ -T \end{pmatrix}, \quad h_{\Delta u} = \begin{pmatrix} \Delta\vec{u}_{max} \otimes \vec{1}_{N_p} \\ \Delta\vec{u}_{max} \otimes \vec{1}_{N_p} \end{pmatrix}$$

**Ràng buộc ZMP** (tuyến tính hóa):

$$x_{ZMP,k+j} = C_{ZMP} \vec{x}_{k+j} = C_{ZMP} (A^j \vec{x}_k + [\mathcal{B} \vec{U}]_j)$$

$$x_{ZMP,min} \leq C_{ZMP} \vec{x}_{k+j} \leq x_{ZMP,max}$$

Với $C_{ZMP} = (1, 0, 0, -z_c/g, 0, 0)$ (từ công thức ZMP, Mục 6.2).

#### 8.7.5 Giải QP — Phương pháp Active Set

Trên STM32, dùng Active Set hoặc ADMM (nhẹ hơn Interior Point).

**Active Set đơn giản:**

1. Bắt đầu với nghiệm unconstrained: $\vec{U}^* = -H^{-1} f$
2. Kiểm tra ràng buộc nào bị vi phạm
3. Thêm ràng buộc vi phạm vào tập "active" (đẳng thức)
4. Giải lại bài toán QP thu nhỏ với active constraints
5. Lặp đến khi không còn vi phạm

**Chi phí tính toán:**
- $H \in \mathbb{R}^{5N_p \times 5N_p}$
- Với $N_p = 10$: H kích thước 50×50
- Cholesky factorization: ~$50^3/3 \approx 42000$ phép nhân
- STM32H743 @ 480MHz, FPU: ~0.1ms
- Mỗi lần lặp active set: ~0.05ms
- Tổng (3–5 lần lặp): **~0.5ms** → khả thi trong 20ms frame

#### 8.7.6 Tham số MPC đề xuất

| Tham số | Ký hiệu | Giá trị | Lý do |
|---------|---------|---------|-------|
| Prediction horizon | $N_p$ | 10 | 10 × 20ms = 200ms nhìn trước |
| Control horizon | $N_c$ | 5 | Giảm biến tối ưu (u cố định sau Nc) |
| Q (roll) | $q_\phi$ | 500 | Ưu tiên roll |
| Q (pitch) | $q_\theta$ | 300 | |
| R (ankle) | $r_{ankle}$ | 1 | Ưu tiên dùng ankle |
| R (hip) | $r_{hip}$ | 3 | |
| S (smooth) | $s_{all}$ | 10 | Phạt thay đổi đột ngột |
| $Q_f$ | = P (DARE) | — | Terminal stability |

#### 8.7.7 Control Horizon — Giảm tính toán

Thay vì tối ưu $N_p$ bước input, chỉ tối ưu $N_c \leq N_p$ bước đầu, giữ nguyên sau đó:

$$\vec{u}_{k+j} = \vec{u}_{k+N_c-1} \quad \forall j \geq N_c$$

Kích thước bài toán QP giảm từ $5N_p$ xuống $5N_c$ biến.

Với $N_c = 5$: H kích thước 25×25 → **~5× nhanh hơn**.

---

### 8.8 Giải QP trên STM32 — ADMM

#### 8.8.1 Tại sao ADMM?

MPC mỗi cycle cần giải 1 bài toán QP. Các phương pháp:

| Phương pháp | Ưu | Nhược |
|-------------|-----|-------|
| Active Set | Chính xác, ít lặp | Worst-case $O(2^n)$ lặp, không dự đoán được |
| Interior Point | Hội tụ nhanh | Cần factorize ma trận lớn mỗi lặp |
| **ADMM** | **Mỗi lặp rẻ, warm-start tốt, thời gian dự đoán được** | Hội tụ chậm hơn (nhưng đủ cho real-time) |

ADMM (Alternating Direction Method of Multipliers) phù hợp nhất cho embedded vì:
- Mỗi lặp chỉ cần 1 phép giải hệ tuyến tính (factorize 1 lần, dùng lại)
- Warm-start từ nghiệm cycle trước → thường 5–15 lặp
- Thời gian worst-case có thể bound → real-time guarantee

#### 8.8.2 Reformulation

Bài toán QP chuẩn (từ Mục 8.7.3):

$$\min_{\vec{U}} \frac{1}{2}\vec{U}^T H \vec{U} + f^T \vec{U} \quad \text{s.t.} \quad G\vec{U} \leq h$$

Chuyển sang dạng ADMM bằng biến phụ $\vec{z}$:

$$\min_{\vec{U}, \vec{z}} \frac{1}{2}\vec{U}^T H \vec{U} + f^T \vec{U} + \mathcal{I}_{\mathcal{C}}(\vec{z})$$

$$\text{s.t.} \quad \vec{U} = \vec{z}$$

Trong đó $\mathcal{I}_\mathcal{C}(\vec{z})$ là indicator function của tập ràng buộc:

$$\mathcal{I}_\mathcal{C}(\vec{z}) = \begin{cases} 0 & \text{nếu } \vec{z} \in \mathcal{C} = \{\vec{z} : G\vec{z} \leq h\} \\ +\infty & \text{ngược lại} \end{cases}$$

#### 8.8.3 Augmented Lagrangian

$$\mathcal{L}_\rho(\vec{U}, \vec{z}, \vec{y}) = \frac{1}{2}\vec{U}^T H \vec{U} + f^T \vec{U} + \mathcal{I}_\mathcal{C}(\vec{z}) + \vec{y}^T(\vec{U} - \vec{z}) + \frac{\rho}{2}\|\vec{U} - \vec{z}\|^2$$

Trong đó:
- $\vec{y}$ = dual variable (Lagrange multiplier)
- $\rho > 0$ = penalty parameter

#### 8.8.4 Các bước lặp ADMM

Scaled form (đặt $\vec{w} = \vec{y}/\rho$):

**Bước 1 — Cập nhật $\vec{U}$ (giải hệ tuyến tính):**

$$\vec{U}^{(i+1)} = (H + \rho I)^{-1}(-f + \rho(\vec{z}^{(i)} - \vec{w}^{(i)}))$$

Ma trận $(H + \rho I)$ **không đổi** giữa các lần lặp → **factorize Cholesky 1 lần**, back-substitution mỗi lặp.

Cholesky: $H + \rho I = LL^T$

Mỗi lặp: giải $LL^T \vec{U} = \vec{r}$ bằng forward + backward substitution.

**Bước 2 — Cập nhật $\vec{z}$ (projection lên tập ràng buộc):**

$$\vec{z}^{(i+1)} = \Pi_\mathcal{C}(\vec{U}^{(i+1)} + \vec{w}^{(i)})$$

Với box constraints ($\vec{u}_{min} \leq \vec{z} \leq \vec{u}_{max}$), projection chỉ là clamping:

$$z_j^{(i+1)} = \text{clamp}(U_j^{(i+1)} + w_j^{(i)}, \ u_{min,j}, \ u_{max,j})$$

**Bước 3 — Cập nhật dual variable:**

$$\vec{w}^{(i+1)} = \vec{w}^{(i)} + \vec{U}^{(i+1)} - \vec{z}^{(i+1)}$$

#### 8.8.5 Điều kiện dừng

Primal residual:

$$r_{prim}^{(i)} = \|\vec{U}^{(i)} - \vec{z}^{(i)}\|_2$$

Dual residual:

$$r_{dual}^{(i)} = \rho \|\vec{z}^{(i)} - \vec{z}^{(i-1)}\|_2$$

Dừng khi:

$$r_{prim}^{(i)} < \epsilon_{abs} + \epsilon_{rel} \cdot \max(\|\vec{U}^{(i)}\|, \|\vec{z}^{(i)}\|)$$

$$r_{dual}^{(i)} < \epsilon_{abs} + \epsilon_{rel} \cdot \rho\|\vec{w}^{(i)}\|$$

Gợi ý: $\epsilon_{abs} = 10^{-3}$, $\epsilon_{rel} = 10^{-3}$.

Hoặc đơn giản: cố định số lần lặp $N_{iter}$ = 10–20 (dễ đảm bảo timing).

#### 8.8.6 Warm-starting

Khởi tạo từ nghiệm MPC cycle trước (shifted):

$$\vec{U}_0^{(k)} = \begin{pmatrix} \vec{u}_{1}^{*(k-1)} \\ \vec{u}_{2}^{*(k-1)} \\ \vdots \\ \vec{u}_{N_c-1}^{*(k-1)} \\ \vec{u}_{N_c-1}^{*(k-1)} \end{pmatrix}$$

Shift lên 1 bước, lặp lại phần tử cuối. Warm-start giảm số lần lặp ADMM từ ~30 xuống ~5–10.

#### 8.8.7 Chọn penalty $\rho$

$\rho$ ảnh hưởng tốc độ hội tụ:
- $\rho$ quá nhỏ → primal residual hội tụ chậm
- $\rho$ quá lớn → dual residual hội tụ chậm

**Quy tắc thực nghiệm:**

$$\rho = \sqrt{\frac{\|H\|_F \cdot \epsilon_{abs}}{\|G\|_F \cdot \epsilon_{abs}}} \approx \sqrt{\frac{\text{trace}(H)}{\text{trace}(G^TG)}}$$

Hoặc adaptive $\rho$ (Residual balancing — Boyd 2011):

$$\rho^{(i+1)} = \begin{cases} \tau \rho^{(i)} & \text{nếu } r_{prim}^{(i)} > \mu \cdot r_{dual}^{(i)} \\ \rho^{(i)} / \tau & \text{nếu } r_{dual}^{(i)} > \mu \cdot r_{prim}^{(i)} \\ \rho^{(i)} & \text{ngược lại} \end{cases}$$

Với $\tau = 2, \mu = 10$. **Lưu ý:** khi $\rho$ thay đổi, phải re-factorize $(H + \rho I)$ → chi phí lớn. Trên STM32, nên dùng $\rho$ cố định.

#### 8.8.8 Chi phí tính toán trên STM32H743

Với $N_c = 5$, 5 inputs → $n_{var} = 25$:

| Bước | Phép tính | Chi phí |
|------|-----------|---------|
| Cholesky (1 lần) | $n^3/3$ | $25^3/3 \approx 5200$ FP mul |
| Back-sub (mỗi lặp) | $2n^2$ | $2 \times 625 = 1250$ FP mul |
| Projection (mỗi lặp) | $n$ clamp | 25 compare |
| Dual update (mỗi lặp) | $n$ add | 25 add |

Tổng với 10 lặp:

$$5200 + 10 \times (1250 + 50) = 18200 \text{ FP operations}$$

STM32H743 FPU @ 480MHz: ~2 FLOP/cycle → **~38μs** cho toàn bộ MPC solve.

Cộng thêm overhead (ma trận B prediction, f vector): **~100μs tổng** → rất thoải mái trong 20ms frame.

---

### 8.9 Adaptive Control — Bù sai số mô hình

#### 8.9.1 Vấn đề

Ma trận A, B trong Mục 8.2 phụ thuộc:
- $z_c$ — thay đổi khi gối gập/duỗi
- $b_{ij}$ — phụ thuộc hình học, sai số lớn do khung thủ công
- $b_\phi, b_\theta$ — hệ số ma sát, thay đổi theo thời gian

Nếu mô hình sai → LQR/MPC tính sai → robot ngã.

#### 8.9.2 Recursive Least Squares (RLS) — Ước lượng online

Mô hình:

$$\vec{x}_{k+1} = A \vec{x}_k + B \vec{u}_k + \vec{w}_k$$

Muốn ước lượng A, B online từ dữ liệu đo được. Viết lại dạng hồi quy:

$$\vec{x}_{k+1} = \Theta^T \vec{z}_k$$

Trong đó:
- $\Theta = [A \ B]^T \in \mathbb{R}^{(n+m) \times n}$ — ma trận tham số cần ước lượng
- $\vec{z}_k = [\vec{x}_k^T \ \vec{u}_k^T]^T \in \mathbb{R}^{n+m}$ — regressor
- n = 6 (trạng thái), m = 5 (input)

**Cập nhật RLS:**

Sai số dự đoán:

$$\vec{e}_k = \vec{x}_{k+1} - \hat{\Theta}_k^T \vec{z}_k$$

Gain:

$$\vec{\ell}_k = \frac{P_k \vec{z}_k}{\lambda + \vec{z}_k^T P_k \vec{z}_k}$$

Cập nhật tham số:

$$\hat{\Theta}_{k+1} = \hat{\Theta}_k + \vec{\ell}_k \vec{e}_k^T$$

Cập nhật covariance:

$$P_{k+1} = \frac{1}{\lambda} \left( P_k - \vec{\ell}_k \vec{z}_k^T P_k \right)$$

Trong đó:
- $\lambda \in (0.95, 1.0]$ — forgetting factor
- $\lambda$ nhỏ → quên nhanh, thích nghi nhanh nhưng nhiễu
- $\lambda$ lớn → ổn định hơn nhưng thích nghi chậm
- Gợi ý: $\lambda = 0.98$ cho robot đi bộ

**Khởi tạo:**
- $\hat{\Theta}_0 = [A_0 \ B_0]^T$ (mô hình danh nghĩa từ Mục 8.3)
- $P_0 = \alpha I_{(n+m) \times (n+m)}$ với $\alpha = 100$ (uncertainty cao ban đầu)

#### 8.9.3 Cập nhật LQR gain online

Khi $\hat{A}_k, \hat{B}_k$ thay đổi đáng kể, giải lại DARE:

$$\text{Nếu } \|\hat{\Theta}_{k} - \hat{\Theta}_{prev}\|_F > \epsilon_{update}:$$
$$\quad \text{Giải DARE}(\hat{A}_k, \hat{B}_k, Q, R) \rightarrow K_{new}$$
$$\quad \hat{\Theta}_{prev} \leftarrow \hat{\Theta}_k$$

Không cần giải DARE mỗi cycle — chỉ khi mô hình thay đổi đủ lớn (tiết kiệm tính toán).

$\epsilon_{update} \approx 0.05$ — ngưỡng thay đổi ~5%.

#### 8.9.4 Disturbance Observer (DOB) — Phương pháp đơn giản hơn

Thay vì ước lượng toàn bộ A, B, chỉ ước lượng **nhiễu** (disturbance) tác động lên hệ:

$$\vec{x}_{k+1} = A \vec{x}_k + B \vec{u}_k + \vec{d}_k$$

Ước lượng nhiễu:

$$\hat{\vec{d}}_k = \vec{x}_{k+1} - A \vec{x}_k - B \vec{u}_k$$

Lọc nhiễu (low-pass vì d thay đổi chậm):

$$\bar{\vec{d}}_k = \alpha_d \cdot \hat{\vec{d}}_k + (1 - \alpha_d) \cdot \bar{\vec{d}}_{k-1}$$

Với $\alpha_d \approx 0.05 \div 0.1$ (lọc mạnh).

Bù nhiễu vào luật điều khiển:

$$\vec{u}_k = -K \vec{x}_k - B^\dagger \bar{\vec{d}}_k$$

Trong đó $B^\dagger = (B^T B)^{-1} B^T$ là pseudo-inverse của B.

**Ưu điểm DOB:** Đơn giản hơn RLS rất nhiều, không cần giải lại DARE, phù hợp cho disturbance chậm (lệch tâm, bias servo).

---

### 8.10 Gain Scheduling theo Gait Phase

#### 8.10.1 Tại sao?

Hệ thống phi tuyến — đặc tính thay đổi theo pha bước:
- **Double support**: robot rất ổn định → gain nhỏ, tránh dao động
- **Early single support**: vừa nhấc chân, trọng tâm đang chuyển → gain trung bình
- **Mid single support**: 1 chân, support polygon nhỏ nhất → gain lớn nhất
- **Late single support**: chuẩn bị hạ chân → giảm gain để smooth landing

#### 8.10.2 Interpolation giữa các bộ gain

Tính offline 3 bộ gain:
- $K_{DS}$ : gain cho double support (Q nhỏ)
- $K_{SS}$ : gain cho single support (Q lớn)
- $K_{TR}$ : gain cho transition (Q trung bình)

Nội suy theo pha:

$$K(\phi) = w_{DS}(\phi) \cdot K_{DS} + w_{SS}(\phi) \cdot K_{SS} + w_{TR}(\phi) \cdot K_{TR}$$

Với trọng số $w_{DS} + w_{SS} + w_{TR} = 1$, phụ thuộc $\phi_{local}$ (Mục 5.2).

Hàm trọng số (raised cosine blend):

Cho $\phi_{local} \in [0, 1]$ và $\rho$ = double support ratio:

$$w_{DS}(\phi) = \begin{cases} \frac{1}{2}\left(1 + \cos\left(\frac{\pi(\phi - 0)}{\rho}\right)\right) & \phi \in [0, \rho] \\ 0 & \phi \in (\rho, 1-\rho) \\ \frac{1}{2}\left(1 + \cos\left(\frac{\pi(1 - \phi)}{\rho}\right)\right) & \phi \in [1-\rho, 1] \end{cases}$$

$$w_{SS}(\phi) = 1 - w_{DS}(\phi)$$

#### 8.10.3 z_c scheduling

Chiều cao trọng tâm thay đổi trong 1 stride (Mục 5.8):

$$z_c(\phi) = z_{c,0} + \Delta z \cos(4\pi\phi)$$

Cập nhật $\omega_n^2(\phi) = g / z_c(\phi)$ trong ma trận A trước khi tính luật điều khiển.

Nếu dùng LQR với K cố định: bỏ qua (sai số nhỏ vì $\Delta z \ll z_{c,0}$).
Nếu dùng MPC: cập nhật A trong prediction model (time-varying MPC).

---

### 8.11 Phân bổ lực — Control Allocation

#### 8.11.1 Bài toán

LQR/MPC cho ra **moment bù mong muốn** $(\tau_{roll}, \tau_{pitch})$. Cần phân bổ vào 5 khớp:

$$\tau_{desired} = B_{alloc} \cdot \vec{u}$$

Với $B_{alloc} \in \mathbb{R}^{2 \times 5}$ — ma trận ảnh hưởng.

Hệ under-determined (2 phương trình, 5 ẩn) → vô số nghiệm → chọn nghiệm **tối ưu**.

#### 8.11.2 Weighted Least-Norm Allocation

$$\min_{\vec{u}} \vec{u}^T W \vec{u} \quad \text{s.t.} \quad B_{alloc} \vec{u} = \tau_{desired}$$

Nghiệm giải tích:

$$\vec{u}^* = W^{-1} B_{alloc}^T (B_{alloc} W^{-1} B_{alloc}^T)^{-1} \tau_{desired}$$

Trong đó $W = R$ (ma trận trọng số từ LQR) — khớp có R lớn sẽ ít bị dùng.

**Ý nghĩa:** Thay vì chia cứng 50%-30%-20% cho ankle/hip/torso (PD approach), allocation tự động tìm phân bổ tối ưu, có thể thay đổi khi 1 khớp bão hòa.

#### 8.11.3 Allocation với ràng buộc bão hòa

Khi khớp chạm giới hạn, loại bỏ khớp đó và giải lại:

1. Giải unconstrained: $\vec{u}^*$
2. Nếu $u_i^* > u_{max,i}$: gán $u_i = u_{max,i}$, loại khớp i
3. Giải lại với ma trận $B_{alloc}$ bỏ cột i, $\tau_{desired}$ trừ đi phần do khớp i bão hòa đóng góp
4. Lặp đến khi không còn bão hòa hoặc hết khớp

---

### 8.12 Tổng hợp góc khớp cuối cùng

Kết hợp tất cả thành phần:

$$\boxed{\theta_i^{final} = \theta_i^{IK} + \theta_i^{ff} + \Delta\theta_i^{LQR/MPC} + \Delta\theta_i^{servo\_fb} + \Delta\theta_i^{DOB}}$$

Trong đó:
- $\theta_i^{IK}$ : góc từ Inverse Kinematics (Mục 4)
- $\theta_i^{ff}$ : feedforward từ dynamic model (Mục 8.2.3)
- $\Delta\theta_i^{LQR/MPC}$ : correction từ LQR hoặc MPC (Mục 8.4/8.6)
- $\Delta\theta_i^{servo\_fb}$ : closed-loop servo compensation (Mục 9)
- $\Delta\theta_i^{DOB}$ : disturbance observer compensation (Mục 8.7.4)

**Ràng buộc cuối:**

$$\theta_i^{final} = \text{clamp}(\theta_i^{final}, \ \theta_{i,min}, \ \theta_{i,max})$$

---

### 8.13 Tổng quan pipeline điều khiển

#### Architecture hoàn chỉnh

$$\boxed{\vec{u}_{total} = \underbrace{B_{ff}^{\dagger}\vec{\tau}_{ff}}_{\text{8.3 Feedforward}} + \underbrace{(-K_{LQI}\vec{x}_{aug})}_{\text{8.4–8.5 LQR/LQI}} + \underbrace{\vec{u}_{MPC}^*}_{\text{8.7 MPC}} + \underbrace{(-B^{\dagger}\bar{\vec{d}})}_{\text{8.9.4 DOB}}}$$

Trong thực tế, chọn **một trong hai**: LQI hoặc MPC (không chồng).

**Cấu hình đề xuất cho PNOID:**

| Tầng | Thành phần | Vai trò | Chi phí |
|------|-----------|---------|---------|
| Feedforward | Inverse dynamics (8.3) | Bù trọng lực + quán tính đã biết | ~10μs |
| Feedback | LQI (8.4–8.5) hoặc MPC (8.7–8.8) | Bù sai lệch, coupling, ràng buộc | ~20μs / ~100μs |
| Adaptation | DOB (8.9.4) | Bù bias khung lệch, servo sai | ~5μs |
| Scheduling | Gain scheduling (8.10) | Điều chỉnh gain theo gait phase | ~2μs |
| Allocation | Control allocation (8.11) | Phân bổ tối ưu vào 5 khớp | ~10μs |
| **Tổng** | | | **~50μs / ~130μs** |

Budget: 20ms frame → sử dụng < 1% CPU → rất thoải mái.

**Lộ trình triển khai:**
1. **FF + LQI + DOB** — triển khai trước, đủ cho đứng và bước tại chỗ
2. **Thêm Gain Scheduling** — khi bắt đầu đi bộ thật
3. **Chuyển LQI → MPC** — khi cần tối ưu hóa ràng buộc (bước nhanh, ZMP tight)
4. **Thêm RLS** — nếu đổi servo hoặc thay đổi khung cơ khí

---

## 9. Phản hồi vị trí servo

### 9.1 Mô hình tín hiệu

Potentiometer trong SG92R tạo điện áp tỷ lệ vị trí:

$$V_{fb} = V_{min} + (V_{max} - V_{min}) \cdot \frac{\theta_{actual}}{180°}$$

Trong đó:
- $V_{min} \approx 0.3 \div 0.5$ V (tại 0°)
- $V_{max} \approx 2.2 \div 2.8$ V (tại 180°)
- Giá trị thay đổi **theo từng servo** → cần calibrate riêng

### 9.2 Chuyển đổi ADC → Góc

ADC 12-bit, $V_{ref}$ = 3.3V:

$$V = \frac{ADC_{raw}}{4095} \cdot 3.3$$

Nội suy tuyến tính từng đoạn (piecewise linear) qua 3 điểm calibration:

Cho 3 điểm calibration $(ADC_0, 0°), (ADC_{90}, 90°), (ADC_{180}, 180°)$:

Nếu $ADC_{raw} \leq ADC_{90}$:

$$\theta_{actual} = 90° \cdot \frac{ADC_{raw} - ADC_0}{ADC_{90} - ADC_0}$$

Nếu $ADC_{raw} > ADC_{90}$:

$$\theta_{actual} = 90° + 90° \cdot \frac{ADC_{raw} - ADC_{90}}{ADC_{180} - ADC_{90}}$$

### 9.3 Lọc nhiễu — Exponential Moving Average

Tín hiệu feedback rất nhiễu. Lọc bằng EMA:

$$\bar{V}_k = \alpha_f \cdot V_k + (1 - \alpha_f) \cdot \bar{V}_{k-1}$$

Với $\alpha_f \approx 0.15 \div 0.3$ (nhỏ = mượt hơn nhưng trễ hơn).

### 9.4 Closed-loop servo compensation

Sai lệch vị trí servo:

$$e_{servo,i} = \theta_i^{target} - \theta_i^{actual}$$

Bù tỷ lệ (P-only, vì servo đã có controller nội bộ):

$$\Delta\theta_i^{servo\_fb} = \begin{cases} K_{fb} \cdot e_{servo,i} & \text{nếu } |e_{servo,i}| > \delta \\ 0 & \text{nếu } |e_{servo,i}| \leq \delta \end{cases}$$

Trong đó:
- $K_{fb} \approx 0.2 \div 0.4$ — gain bù
- $\delta \approx 3°$ — dead-band (bỏ qua sai số nhỏ hơn nhiễu đo)

### 9.5 Phát hiện quá tải / kẹt

Nếu sai lệch lớn kéo dài → servo bị kẹt hoặc quá tải:

$$\text{stall detected if: } |e_{servo,i}| > \theta_{stall} \text{ trong } N_{stall} \text{ cycles liên tiếp}$$

Gợi ý: $\theta_{stall} = 15°$, $N_{stall} = 25$ cycles (= 0.5s tại 50Hz).

Phản ứng: giảm tốc hoặc dừng gait để bảo vệ servo.

---

## 10. Mô hình con lắc ngược (LIPM)

### 10.1 Linear Inverted Pendulum Model

Đơn giản hóa robot thành khối lượng m tại chiều cao z_c, chân cứng:

```
            CoM (m)
           ╱
          ╱  z_c
         ╱
  ──────●──────── mặt đất
       ZMP
```

Phương trình chuyển động:

$$\ddot{x}_c = \frac{g}{z_c}(x_c - x_{ZMP})$$

$$\ddot{y}_c = \frac{g}{z_c}(y_c - y_{ZMP})$$

### 10.2 Nghiệm tổng quát

Đây là phương trình vi phân tuyến tính bậc 2, nghiệm:

$$x_c(t) = C_1 e^{\omega_n t} + C_2 e^{-\omega_n t} + x_{ZMP}$$

Trong đó:

$$\omega_n = \sqrt{\frac{g}{z_c}}$$

Ví dụ: $z_c = 100$ mm = 0.1 m → $\omega_n = \sqrt{9.81/0.1} \approx 9.9$ rad/s → rất nhanh, robot nhỏ khó giữ thăng bằng.

### 10.3 Ý nghĩa vật lý

- Nếu $x_c > x_{ZMP}$: CoM phía trước ZMP → $\ddot{x}_c > 0$ → ngã thêm về phía trước
- Nếu $x_c < x_{ZMP}$: CoM phía sau ZMP → $\ddot{x}_c < 0$ → ngã về phía sau
- **Hệ thống bất ổn định** (unstable) — cần điều khiển chủ động

### 10.4 Capture Point

Capture Point là vị trí mà nếu đặt chân tại đó, robot sẽ dừng lại:

$$x_{capture} = x_c + \frac{\dot{x}_c}{\omega_n}$$

$$y_{capture} = y_c + \frac{\dot{y}_c}{\omega_n}$$

**Ứng dụng:** Khi robot sắp ngã, đặt chân swing tại capture point để hồi phục.

### 10.5 Preview Control (nâng cao)

Thay vì reactive (phản ứng sau khi lệch), preview control nhìn trước quỹ đạo ZMP mong muốn và tính quỹ đạo CoM tối ưu.

Rời rạc hóa LIPM:

$$\vec{x}_{k+1} = A \vec{x}_k + B u_k$$

$$p_k = C \vec{x}_k$$

Với trạng thái $\vec{x} = (x_c, \dot{x}_c, \ddot{x}_c)^T$, đầu vào $u = \dddot{x}_c$ (jerk), đầu ra $p = x_{ZMP}$:

$$A = \begin{pmatrix} 1 & \Delta t & \Delta t^2/2 \\ 0 & 1 & \Delta t \\ 0 & 0 & 1 \end{pmatrix}, \quad B = \begin{pmatrix} \Delta t^3/6 \\ \Delta t^2/2 \\ \Delta t \end{pmatrix}$$

$$C = \begin{pmatrix} 1 & 0 & -z_c/g \end{pmatrix}$$

Hàm chi phí tối ưu:

$$J = \sum_{k=0}^{N} Q_e \left(p_k^{ref} - p_k\right)^2 + R \cdot u_k^2$$

Nghiệm optimal preview controller (Kajita 2003):

$$u_k = -K_x \vec{x}_k - \sum_{j=1}^{N_L} f_j \cdot p_{k+j}^{ref}$$

Trong đó:
- $K_x$ = state feedback gain (giải Riccati)
- $f_j$ = preview gain cho $N_L$ bước nhìn trước
- $p_{k+j}^{ref}$ = quỹ đạo ZMP tham chiếu tương lai

**Cho robot SG92R:** Preview control là lý thuyết lý tưởng nhưng khó áp dụng trực tiếp vì servo chậm và không chính xác. Nên dùng như mục tiêu dài hạn, bắt đầu với reactive stabilizer (Mục 8).

---

## 11. Tham số và điều kiện biên

### 11.1 Bảng tham số hình học (đo thực tế)

| Tham số | Ký hiệu | Giá trị ước lượng | Đơn vị |
|---------|---------|-------------------|--------|
| Nửa khoảng cách 2 hông | d | 35 | mm |
| Offset pelvis-hip dọc | h_z | 15 | mm |
| Chiều dài đùi | L₁ | 55 | mm |
| Chiều dài ống chân | L₂ | 55 | mm |
| Chiều cao bàn chân | h₀ | 20 | mm |

### 11.2 Bảng giới hạn khớp

| Khớp | θ_min | θ_max | Ghi chú |
|------|-------|-------|---------|
| HipYaw (θ₁) | −45° | +45° | |
| HipRoll (θ₂) | −30° | +30° | |
| HipPitch (θ₃) | −90° | +45° | Âm = đùi lên trước |
| KneePitch (θ₄) | +15° | +135° | Min 15° = bent-knee |
| AnklePitch (θ₅) | −45° | +45° | |
| AnkleRoll (θ₆) | −30° | +30° | |
| TorsoYaw (θ_T1) | −45° | +45° | |
| TorsoRoll (θ_T2) | −30° | +30° | |

### 11.3 Bảng tham số gait (khởi đầu)

| Tham số | Ký hiệu | Giá trị | Đơn vị | Phạm vi tune |
|---------|---------|---------|--------|-------------|
| Chiều dài bước | S | 20 | mm | 10 – 60 |
| Chiều cao nhấc chân | H | 15 | mm | 10 – 40 |
| Thời gian 1 bước | T_step | 800 | ms | 400 – 1500 |
| Chiều cao trọng tâm | z_c | 90 | mm | 70 – 110 |
| Biên độ lắc ngang | W | 15 | mm | 10 – 30 |
| Double support ratio | ρ | 0.15 | — | 0.05 – 0.30 |
| Biên độ nhún dọc | Δz | 3 | mm | 1 – 5 |
| Gối gập tối thiểu | θ₄,min | 20 | ° | 10 – 35 |

### 11.4 Bảng tham số stabilizer (LQR/LQI)

| Tham số | Ký hiệu | Giá trị khởi đầu | Ghi chú |
|---------|---------|------------------|---------|
| Q — roll weight | q_φ | 500 | Ưu tiên roll (support polygon hẹp ngang) |
| Q — pitch weight | q_θ | 300 | |
| Q — roll rate | q_dφ | 10 | Damping |
| Q — pitch rate | q_dθ | 10 | |
| Q — integral roll | q_ξφ | 50 | Triệt tiêu steady-state error |
| Q — integral pitch | q_ξθ | 30 | |
| R — ankle | r_ankle | 1 | Ưu tiên dùng ankle (nhanh nhất) |
| R — hip | r_hip | 3 | |
| R — torso | r_torso | 5 | Hạn chế giật thân trên |
| Comp. filter α | α | 0.98 | τ ≈ 1.0s cho Δt = 20ms |
| DOB filter | α_d | 0.05 | Lọc mạnh (disturbance chậm) |
| Servo fb gain | K_fb | 0.2 | |
| Servo fb deadband | δ | 3 | ° |
| Max correction | Δθ_max | 12 | ° |
| Forgetting factor (RLS) | λ | 0.98 | Adaptive, nếu dùng |

### 11.4b Bảng tham số MPC (nếu dùng thay LQI)

| Tham số | Ký hiệu | Giá trị | Ghi chú |
|---------|---------|---------|---------|
| Prediction horizon | N_p | 10 | 200ms nhìn trước |
| Control horizon | N_c | 5 | Giảm kích thước QP |
| Smooth penalty | S | diag(10) | Phạt thay đổi input đột ngột |
| Terminal cost | Q_f | P (DARE) | Đảm bảo ổn định |
| ADMM penalty | ρ | 10 | Cố định trên STM32 |
| ADMM max iter | N_iter | 15 | Giới hạn thời gian worst-case |
| ADMM tolerance | ε_abs | 10⁻³ | |

### 11.5 Bảng tham số servo SG92R

| Tham số | Giá trị |
|---------|---------|
| Tần số PWM | 50 Hz |
| Pulse range | 500 – 2500 μs |
| Góc hoạt động | 0° – 180° |
| Tốc độ (không tải) | ~0.1 s / 60° |
| Dead-band | ~6–7 μs |
| Moment xoắn | ~1.8 kgf·cm |
| Điện áp hoạt động | 4.8 – 6.0 V |

### 11.6 Tần số tự nhiên LIPM

$$\omega_n = \sqrt{\frac{g}{z_c}}$$

| z_c (mm) | ω_n (rad/s) | Chu kỳ dao động tự nhiên (s) |
|-----------|------------|------------------------------|
| 70 | 11.8 | 0.53 |
| 90 | 10.4 | 0.60 |
| 110 | 9.4 | 0.67 |

Robot nhỏ (z_c thấp) → ω_n lớn → cần phản ứng nhanh → servo phải đáp ứng trong vài chục ms.

Với SG92R tốc độ 0.1s/60° = 600°/s, correction 10° mất ~17ms → vừa đủ nếu control loop = 20ms.

---

## Phụ lục A: Chứng minh Cosine Rule cho IK

Tam giác Hip-Knee-Ankle với cạnh:
- a = L₁ (đùi)
- b = L₂ (ống chân)
- c = L (khoảng cách hip–ankle)

Góc tại Knee:

$$c^2 = a^2 + b^2 - 2ab\cos(\alpha_k)$$

$$\cos(\alpha_k) = \frac{a^2 + b^2 - c^2}{2ab} = \frac{L_1^2 + L_2^2 - L^2}{2 L_1 L_2}$$

$\alpha_k$ là góc **bên trong** tam giác. Góc gập gối (θ₄) là góc bù:

$$\theta_4 = \pi - \alpha_k = \pi - \arccos\!\left(\frac{L_1^2 + L_2^2 - L^2}{2 L_1 L_2}\right)$$

Khi chân duỗi thẳng: $L = L_1 + L_2$, $\cos\alpha_k = \frac{L_1^2 + L_2^2 - (L_1+L_2)^2}{2L_1L_2} = \frac{-2L_1L_2}{2L_1L_2} = -1$, $\alpha_k = \pi$, $\theta_4 = 0$. ✓

Khi gập hoàn toàn: $L = |L_1 - L_2|$, $\theta_4 = \pi$. (Không bao giờ đạt do giới hạn cơ khí.)

---

## Phụ lục B: Chuyển đổi Euler ↔ Rotation Matrix

### Euler ZYX (yaw-pitch-roll):

$$R = R_z(\psi) \cdot R_y(\theta) \cdot R_x(\phi)$$

$$= \begin{pmatrix} c\psi c\theta & c\psi s\theta s\phi - s\psi c\phi & c\psi s\theta c\phi + s\psi s\phi \\ s\psi c\theta & s\psi s\theta s\phi + c\psi c\phi & s\psi s\theta c\phi - c\psi s\phi \\ -s\theta & c\theta s\phi & c\theta c\phi \end{pmatrix}$$

Trong đó $c = \cos, s = \sin$.

### Trích xuất Euler từ R:

$$\theta = -\arcsin(R_{31})$$

$$\phi = \text{atan2}(R_{32}, R_{33})$$

$$\psi = \text{atan2}(R_{21}, R_{11})$$

(Suy biến khi $\theta = \pm\pi/2$ — gimbal lock.)

---

## Phụ lục C: Đạo hàm quỹ đạo Cycloid

Quỹ đạo cycloid cải tiến:

$$x(s) = S\left(s - \frac{\sin(2\pi s)}{2\pi}\right) - \frac{S}{2}$$

Đạo hàm bậc 1 (vận tốc, theo s):

$$\frac{dx}{ds} = S\left(1 - \cos(2\pi s)\right)$$

Kiểm tra:
- $s = 0$: $\frac{dx}{ds} = S(1 - 1) = 0$ ✓
- $s = 0.5$: $\frac{dx}{ds} = S(1 - (-1)) = 2S$ (vận tốc cực đại)
- $s = 1$: $\frac{dx}{ds} = S(1 - 1) = 0$ ✓

Đạo hàm bậc 2 (gia tốc, theo s):

$$\frac{d^2x}{ds^2} = 2\pi S \sin(2\pi s)$$

Liên tục, không có điểm gãy → servo chuyển động mượt.

Chuyển sang miền thời gian với $s = (t - t_0) / T_{swing}$:

$$\dot{x}(t) = \frac{S}{T_{swing}}(1 - \cos(2\pi s))$$

$$\ddot{x}(t) = \frac{2\pi S}{T_{swing}^2}\sin(2\pi s)$$

---

## Phụ lục D: Tham khảo

1. **Kajita, S. et al.** "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point." *ICRA 2003.* — Paper kinh điển về ZMP preview control.
2. **Kajita, S.** "Introduction to Humanoid Robotics." Springer, 2014. — Sách giáo khoa về lý thuyết đi bộ humanoid.
3. **Vukobratović, M.** "Zero-Moment Point — Thirty Five Years of Its Life." *Int. J. Humanoid Robotics, 2004.* — Lịch sử và lý thuyết ZMP.
4. **ROBOTIS Darwin-OP Walking Engine** — Mã nguồn mở, triển khai LIPM + ZMP cho robot nhỏ.
5. **Poppy Project** — Robot humanoid mã nguồn mở dùng servo giá rẻ.
6. **Pratt, J. et al.** "Capture Point: A Step toward Humanoid Push Recovery." *IO Humanoids, 2006.* — Lý thuyết Capture Point.
