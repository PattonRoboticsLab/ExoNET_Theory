function q = computeJointAnglesRobust(shoulder, elbow, wrist)
u = elbow - shoulder;  % upper arm
v = wrist - elbow;     % forearm

u = u / norm(u);
v = v / norm(v);

% --- 2. SHOULDER YAW (rotation in horizontal plane around Z) ---
yaw = atan2(u(2), u(1));  % yaw rispetto a +Y
yaw = mod(yaw, 2*pi);   % normalizza tra 0 e 2π

% --- 3. SHOULDER PITCH (angle from Z axis) ---
z = [0 0 1];  % Asse Z verso l’alto
pitch = atan2(norm(cross(u, z)), dot(u, z));  % Angolo tra u e Z
pitch = mod(pitch, 2*pi);  % Normalizza tra 0 e 2π (opzionale)

% --- 4. SHOULDER ROLL (rotation of forearm around upper arm axis) ---
x_axis = u / norm(u);  % asse del braccio superiore

% Vettore di riferimento (es. -Z)
ref = [0 0 -1];

% Proietta il vettore di riferimento sul piano ortogonale a x_axis
ref_proj = ref - dot(ref, x_axis) * x_axis;
if norm(ref_proj) < 1e-6
    ref_proj = [0 1 0];  % fallback se ortogonale nullo
end
ref_proj = ref_proj / norm(ref_proj);

% Proietta anche il vettore avambraccio v sullo stesso piano
v_proj = v - dot(v, x_axis) * x_axis;
v_proj = v_proj / norm(v_proj);

% Calcolo del roll con atan2
roll = atan2(norm(cross(ref_proj, v_proj)), dot(ref_proj, v_proj));
roll = mod(roll, 2*pi);  % per avere [0, 2π]

% --- 5. ELBOW FLEXION (angle between upper arm and forearm) ---
elbow = acos(dot(u, v));  % elbow flexion angle ∈ [0, pi]

% --- 6. Output (unwrapped version, no mod)
q = [yaw, pitch, roll, elbow];
end