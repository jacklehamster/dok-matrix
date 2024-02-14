// /Users/vincent/dok-matrix/example/node_modules/dok-matrix/dist/index.js
var B0 = function() {
  var K = new S(9);
  if (S != Float32Array)
    K[1] = 0, K[2] = 0, K[3] = 0, K[5] = 0, K[6] = 0, K[7] = 0;
  return K[0] = 1, K[4] = 1, K[8] = 1, K;
};
var $K = function() {
  var K = new S(16);
  if (S != Float32Array)
    K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0;
  return K[0] = 1, K[5] = 1, K[10] = 1, K[15] = 1, K;
};
var NK = function(K) {
  var W = new S(16);
  return W[0] = K[0], W[1] = K[1], W[2] = K[2], W[3] = K[3], W[4] = K[4], W[5] = K[5], W[6] = K[6], W[7] = K[7], W[8] = K[8], W[9] = K[9], W[10] = K[10], W[11] = K[11], W[12] = K[12], W[13] = K[13], W[14] = K[14], W[15] = K[15], W;
};
var QK = function(K, W) {
  return K[0] = W[0], K[1] = W[1], K[2] = W[2], K[3] = W[3], K[4] = W[4], K[5] = W[5], K[6] = W[6], K[7] = W[7], K[8] = W[8], K[9] = W[9], K[10] = W[10], K[11] = W[11], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15], K;
};
var XK = function(K, W, $, N, Q, X, Y, B, O, Z, G, V, H, C, L, E) {
  var J = new S(16);
  return J[0] = K, J[1] = W, J[2] = $, J[3] = N, J[4] = Q, J[5] = X, J[6] = Y, J[7] = B, J[8] = O, J[9] = Z, J[10] = G, J[11] = V, J[12] = H, J[13] = C, J[14] = L, J[15] = E, J;
};
var YK = function(K, W, $, N, Q, X, Y, B, O, Z, G, V, H, C, L, E, J) {
  return K[0] = W, K[1] = $, K[2] = N, K[3] = Q, K[4] = X, K[5] = Y, K[6] = B, K[7] = O, K[8] = Z, K[9] = G, K[10] = V, K[11] = H, K[12] = C, K[13] = L, K[14] = E, K[15] = J, K;
};
var O0 = function(K) {
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var BK = function(K, W) {
  if (K === W) {
    var $ = W[1], N = W[2], Q = W[3], X = W[6], Y = W[7], B = W[11];
    K[1] = W[4], K[2] = W[8], K[3] = W[12], K[4] = $, K[6] = W[9], K[7] = W[13], K[8] = N, K[9] = X, K[11] = W[14], K[12] = Q, K[13] = Y, K[14] = B;
  } else
    K[0] = W[0], K[1] = W[4], K[2] = W[8], K[3] = W[12], K[4] = W[1], K[5] = W[5], K[6] = W[9], K[7] = W[13], K[8] = W[2], K[9] = W[6], K[10] = W[10], K[11] = W[14], K[12] = W[3], K[13] = W[7], K[14] = W[11], K[15] = W[15];
  return K;
};
var OK = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = W[4], B = W[5], O = W[6], Z = W[7], G = W[8], V = W[9], H = W[10], C = W[11], L = W[12], E = W[13], J = W[14], h = W[15], k = $ * B - N * Y, A = $ * O - Q * Y, P = $ * Z - X * Y, I = N * O - Q * B, U = N * Z - X * B, g = Q * Z - X * O, _ = G * E - V * L, j = G * J - H * L, M = G * h - C * L, p = V * J - H * E, F = V * h - C * E, q = H * h - C * J, T = k * q - A * F + P * p + I * M - U * j + g * _;
  if (!T)
    return null;
  return T = 1 / T, K[0] = (B * q - O * F + Z * p) * T, K[1] = (Q * F - N * q - X * p) * T, K[2] = (E * g - J * U + h * I) * T, K[3] = (H * U - V * g - C * I) * T, K[4] = (O * M - Y * q - Z * j) * T, K[5] = ($ * q - Q * M + X * j) * T, K[6] = (J * P - L * g - h * A) * T, K[7] = (G * g - H * P + C * A) * T, K[8] = (Y * F - B * M + Z * _) * T, K[9] = (N * M - $ * F - X * _) * T, K[10] = (L * U - E * P + h * k) * T, K[11] = (V * P - G * U - C * k) * T, K[12] = (B * j - Y * p - O * _) * T, K[13] = ($ * p - N * j + Q * _) * T, K[14] = (E * A - L * I - J * k) * T, K[15] = (G * I - V * A + H * k) * T, K;
};
var ZK = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = W[4], B = W[5], O = W[6], Z = W[7], G = W[8], V = W[9], H = W[10], C = W[11], L = W[12], E = W[13], J = W[14], h = W[15];
  return K[0] = B * (H * h - C * J) - V * (O * h - Z * J) + E * (O * C - Z * H), K[1] = -(N * (H * h - C * J) - V * (Q * h - X * J) + E * (Q * C - X * H)), K[2] = N * (O * h - Z * J) - B * (Q * h - X * J) + E * (Q * Z - X * O), K[3] = -(N * (O * C - Z * H) - B * (Q * C - X * H) + V * (Q * Z - X * O)), K[4] = -(Y * (H * h - C * J) - G * (O * h - Z * J) + L * (O * C - Z * H)), K[5] = $ * (H * h - C * J) - G * (Q * h - X * J) + L * (Q * C - X * H), K[6] = -($ * (O * h - Z * J) - Y * (Q * h - X * J) + L * (Q * Z - X * O)), K[7] = $ * (O * C - Z * H) - Y * (Q * C - X * H) + G * (Q * Z - X * O), K[8] = Y * (V * h - C * E) - G * (B * h - Z * E) + L * (B * C - Z * V), K[9] = -($ * (V * h - C * E) - G * (N * h - X * E) + L * (N * C - X * V)), K[10] = $ * (B * h - Z * E) - Y * (N * h - X * E) + L * (N * Z - X * B), K[11] = -($ * (B * C - Z * V) - Y * (N * C - X * V) + G * (N * Z - X * B)), K[12] = -(Y * (V * J - H * E) - G * (B * J - O * E) + L * (B * H - O * V)), K[13] = $ * (V * J - H * E) - G * (N * J - Q * E) + L * (N * H - Q * V), K[14] = -($ * (B * J - O * E) - Y * (N * J - Q * E) + L * (N * O - Q * B)), K[15] = $ * (B * H - O * V) - Y * (N * H - Q * V) + G * (N * O - Q * B), K;
};
var GK = function(K) {
  var W = K[0], $ = K[1], N = K[2], Q = K[3], X = K[4], Y = K[5], B = K[6], O = K[7], Z = K[8], G = K[9], V = K[10], H = K[11], C = K[12], L = K[13], E = K[14], J = K[15], h = W * Y - $ * X, k = W * B - N * X, A = W * O - Q * X, P = $ * B - N * Y, I = $ * O - Q * Y, U = N * O - Q * B, g = Z * L - G * C, _ = Z * E - V * C, j = Z * J - H * C, M = G * E - V * L, p = G * J - H * L, F = V * J - H * E;
  return h * F - k * p + A * M + P * j - I * _ + U * g;
};
var Z0 = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2], Y = W[3], B = W[4], O = W[5], Z = W[6], G = W[7], V = W[8], H = W[9], C = W[10], L = W[11], E = W[12], J = W[13], h = W[14], k = W[15], A = $[0], P = $[1], I = $[2], U = $[3];
  return K[0] = A * N + P * B + I * V + U * E, K[1] = A * Q + P * O + I * H + U * J, K[2] = A * X + P * Z + I * C + U * h, K[3] = A * Y + P * G + I * L + U * k, A = $[4], P = $[5], I = $[6], U = $[7], K[4] = A * N + P * B + I * V + U * E, K[5] = A * Q + P * O + I * H + U * J, K[6] = A * X + P * Z + I * C + U * h, K[7] = A * Y + P * G + I * L + U * k, A = $[8], P = $[9], I = $[10], U = $[11], K[8] = A * N + P * B + I * V + U * E, K[9] = A * Q + P * O + I * H + U * J, K[10] = A * X + P * Z + I * C + U * h, K[11] = A * Y + P * G + I * L + U * k, A = $[12], P = $[13], I = $[14], U = $[15], K[12] = A * N + P * B + I * V + U * E, K[13] = A * Q + P * O + I * H + U * J, K[14] = A * X + P * Z + I * C + U * h, K[15] = A * Y + P * G + I * L + U * k, K;
};
var JK = function(K, W, $) {
  var N = $[0], Q = $[1], X = $[2], Y, B, O, Z, G, V, H, C, L, E, J, h;
  if (W === K)
    K[12] = W[0] * N + W[4] * Q + W[8] * X + W[12], K[13] = W[1] * N + W[5] * Q + W[9] * X + W[13], K[14] = W[2] * N + W[6] * Q + W[10] * X + W[14], K[15] = W[3] * N + W[7] * Q + W[11] * X + W[15];
  else
    Y = W[0], B = W[1], O = W[2], Z = W[3], G = W[4], V = W[5], H = W[6], C = W[7], L = W[8], E = W[9], J = W[10], h = W[11], K[0] = Y, K[1] = B, K[2] = O, K[3] = Z, K[4] = G, K[5] = V, K[6] = H, K[7] = C, K[8] = L, K[9] = E, K[10] = J, K[11] = h, K[12] = Y * N + G * Q + L * X + W[12], K[13] = B * N + V * Q + E * X + W[13], K[14] = O * N + H * Q + J * X + W[14], K[15] = Z * N + C * Q + h * X + W[15];
  return K;
};
var VK = function(K, W, $) {
  var N = $[0], Q = $[1], X = $[2];
  return K[0] = W[0] * N, K[1] = W[1] * N, K[2] = W[2] * N, K[3] = W[3] * N, K[4] = W[4] * Q, K[5] = W[5] * Q, K[6] = W[6] * Q, K[7] = W[7] * Q, K[8] = W[8] * X, K[9] = W[9] * X, K[10] = W[10] * X, K[11] = W[11] * X, K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15], K;
};
var CK = function(K, W, $, N) {
  var Q = N[0], X = N[1], Y = N[2], B = Math.hypot(Q, X, Y), O, Z, G, V, H, C, L, E, J, h, k, A, P, I, U, g, _, j, M, p, F, q, T, f;
  if (B < D)
    return null;
  if (B = 1 / B, Q *= B, X *= B, Y *= B, O = Math.sin($), Z = Math.cos($), G = 1 - Z, V = W[0], H = W[1], C = W[2], L = W[3], E = W[4], J = W[5], h = W[6], k = W[7], A = W[8], P = W[9], I = W[10], U = W[11], g = Q * Q * G + Z, _ = X * Q * G + Y * O, j = Y * Q * G - X * O, M = Q * X * G - Y * O, p = X * X * G + Z, F = Y * X * G + Q * O, q = Q * Y * G + X * O, T = X * Y * G - Q * O, f = Y * Y * G + Z, K[0] = V * g + E * _ + A * j, K[1] = H * g + J * _ + P * j, K[2] = C * g + h * _ + I * j, K[3] = L * g + k * _ + U * j, K[4] = V * M + E * p + A * F, K[5] = H * M + J * p + P * F, K[6] = C * M + h * p + I * F, K[7] = L * M + k * p + U * F, K[8] = V * q + E * T + A * f, K[9] = H * q + J * T + P * f, K[10] = C * q + h * T + I * f, K[11] = L * q + k * T + U * f, W !== K)
    K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K;
};
var HK = function(K, W, $) {
  var N = Math.sin($), Q = Math.cos($), X = W[4], Y = W[5], B = W[6], O = W[7], Z = W[8], G = W[9], V = W[10], H = W[11];
  if (W !== K)
    K[0] = W[0], K[1] = W[1], K[2] = W[2], K[3] = W[3], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[4] = X * Q + Z * N, K[5] = Y * Q + G * N, K[6] = B * Q + V * N, K[7] = O * Q + H * N, K[8] = Z * Q - X * N, K[9] = G * Q - Y * N, K[10] = V * Q - B * N, K[11] = H * Q - O * N, K;
};
var EK = function(K, W, $) {
  var N = Math.sin($), Q = Math.cos($), X = W[0], Y = W[1], B = W[2], O = W[3], Z = W[8], G = W[9], V = W[10], H = W[11];
  if (W !== K)
    K[4] = W[4], K[5] = W[5], K[6] = W[6], K[7] = W[7], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[0] = X * Q - Z * N, K[1] = Y * Q - G * N, K[2] = B * Q - V * N, K[3] = O * Q - H * N, K[8] = X * N + Z * Q, K[9] = Y * N + G * Q, K[10] = B * N + V * Q, K[11] = O * N + H * Q, K;
};
var LK = function(K, W, $) {
  var N = Math.sin($), Q = Math.cos($), X = W[0], Y = W[1], B = W[2], O = W[3], Z = W[4], G = W[5], V = W[6], H = W[7];
  if (W !== K)
    K[8] = W[8], K[9] = W[9], K[10] = W[10], K[11] = W[11], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[0] = X * Q + Z * N, K[1] = Y * Q + G * N, K[2] = B * Q + V * N, K[3] = O * Q + H * N, K[4] = Z * Q - X * N, K[5] = G * Q - Y * N, K[6] = V * Q - B * N, K[7] = H * Q - O * N, K;
};
var hK = function(K, W) {
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = W[0], K[13] = W[1], K[14] = W[2], K[15] = 1, K;
};
var IK = function(K, W) {
  return K[0] = W[0], K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = W[1], K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = W[2], K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var UK = function(K, W, $) {
  var N = $[0], Q = $[1], X = $[2], Y = Math.hypot(N, Q, X), B, O, Z;
  if (Y < D)
    return null;
  return Y = 1 / Y, N *= Y, Q *= Y, X *= Y, B = Math.sin(W), O = Math.cos(W), Z = 1 - O, K[0] = N * N * Z + O, K[1] = Q * N * Z + X * B, K[2] = X * N * Z - Q * B, K[3] = 0, K[4] = N * Q * Z - X * B, K[5] = Q * Q * Z + O, K[6] = X * Q * Z + N * B, K[7] = 0, K[8] = N * X * Z + Q * B, K[9] = Q * X * Z - N * B, K[10] = X * X * Z + O, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var PK = function(K, W) {
  var $ = Math.sin(W), N = Math.cos(W);
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = N, K[6] = $, K[7] = 0, K[8] = 0, K[9] = -$, K[10] = N, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var AK = function(K, W) {
  var $ = Math.sin(W), N = Math.cos(W);
  return K[0] = N, K[1] = 0, K[2] = -$, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = $, K[9] = 0, K[10] = N, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var DK = function(K, W) {
  var $ = Math.sin(W), N = Math.cos(W);
  return K[0] = N, K[1] = $, K[2] = 0, K[3] = 0, K[4] = -$, K[5] = N, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var G0 = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2], Y = W[3], B = N + N, O = Q + Q, Z = X + X, G = N * B, V = N * O, H = N * Z, C = Q * O, L = Q * Z, E = X * Z, J = Y * B, h = Y * O, k = Y * Z;
  return K[0] = 1 - (C + E), K[1] = V + k, K[2] = H - h, K[3] = 0, K[4] = V - k, K[5] = 1 - (G + E), K[6] = L + J, K[7] = 0, K[8] = H + h, K[9] = L - J, K[10] = 1 - (G + C), K[11] = 0, K[12] = $[0], K[13] = $[1], K[14] = $[2], K[15] = 1, K;
};
var kK = function(K, W) {
  var $ = new S(3), N = -W[0], Q = -W[1], X = -W[2], Y = W[3], B = W[4], O = W[5], Z = W[6], G = W[7], V = N * N + Q * Q + X * X + Y * Y;
  if (V > 0)
    $[0] = (B * Y + G * N + O * X - Z * Q) * 2 / V, $[1] = (O * Y + G * Q + Z * N - B * X) * 2 / V, $[2] = (Z * Y + G * X + B * Q - O * N) * 2 / V;
  else
    $[0] = (B * Y + G * N + O * X - Z * Q) * 2, $[1] = (O * Y + G * Q + Z * N - B * X) * 2, $[2] = (Z * Y + G * X + B * Q - O * N) * 2;
  return G0(K, W, $), K;
};
var TK = function(K, W) {
  return K[0] = W[12], K[1] = W[13], K[2] = W[14], K;
};
var J0 = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[4], Y = W[5], B = W[6], O = W[8], Z = W[9], G = W[10];
  return K[0] = Math.hypot($, N, Q), K[1] = Math.hypot(X, Y, B), K[2] = Math.hypot(O, Z, G), K;
};
var RK = function(K, W) {
  var $ = new S(3);
  J0($, W);
  var N = 1 / $[0], Q = 1 / $[1], X = 1 / $[2], Y = W[0] * N, B = W[1] * Q, O = W[2] * X, Z = W[4] * N, G = W[5] * Q, V = W[6] * X, H = W[8] * N, C = W[9] * Q, L = W[10] * X, E = Y + G + L, J = 0;
  if (E > 0)
    J = Math.sqrt(E + 1) * 2, K[3] = 0.25 * J, K[0] = (V - C) / J, K[1] = (H - O) / J, K[2] = (B - Z) / J;
  else if (Y > G && Y > L)
    J = Math.sqrt(1 + Y - G - L) * 2, K[3] = (V - C) / J, K[0] = 0.25 * J, K[1] = (B + Z) / J, K[2] = (H + O) / J;
  else if (G > L)
    J = Math.sqrt(1 + G - Y - L) * 2, K[3] = (H - O) / J, K[0] = (B + Z) / J, K[1] = 0.25 * J, K[2] = (V + C) / J;
  else
    J = Math.sqrt(1 + L - Y - G) * 2, K[3] = (B - Z) / J, K[0] = (H + O) / J, K[1] = (V + C) / J, K[2] = 0.25 * J;
  return K;
};
var SK = function(K, W, $, N) {
  var Q = W[0], X = W[1], Y = W[2], B = W[3], O = Q + Q, Z = X + X, G = Y + Y, V = Q * O, H = Q * Z, C = Q * G, L = X * Z, E = X * G, J = Y * G, h = B * O, k = B * Z, A = B * G, P = N[0], I = N[1], U = N[2];
  return K[0] = (1 - (L + J)) * P, K[1] = (H + A) * P, K[2] = (C - k) * P, K[3] = 0, K[4] = (H - A) * I, K[5] = (1 - (V + J)) * I, K[6] = (E + h) * I, K[7] = 0, K[8] = (C + k) * U, K[9] = (E - h) * U, K[10] = (1 - (V + L)) * U, K[11] = 0, K[12] = $[0], K[13] = $[1], K[14] = $[2], K[15] = 1, K;
};
var _K = function(K, W, $, N, Q) {
  var X = W[0], Y = W[1], B = W[2], O = W[3], Z = X + X, G = Y + Y, V = B + B, H = X * Z, C = X * G, L = X * V, E = Y * G, J = Y * V, h = B * V, k = O * Z, A = O * G, P = O * V, I = N[0], U = N[1], g = N[2], _ = Q[0], j = Q[1], M = Q[2], p = (1 - (E + h)) * I, F = (C + P) * I, q = (L - A) * I, T = (C - P) * U, f = (1 - (H + h)) * U, i = (J + k) * U, l = (L + A) * g, X0 = (J - k) * g, Y0 = (1 - (H + E)) * g;
  return K[0] = p, K[1] = F, K[2] = q, K[3] = 0, K[4] = T, K[5] = f, K[6] = i, K[7] = 0, K[8] = l, K[9] = X0, K[10] = Y0, K[11] = 0, K[12] = $[0] + _ - (p * _ + T * j + l * M), K[13] = $[1] + j - (F * _ + f * j + X0 * M), K[14] = $[2] + M - (q * _ + i * j + Y0 * M), K[15] = 1, K;
};
var jK = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = $ + $, B = N + N, O = Q + Q, Z = $ * Y, G = N * Y, V = N * B, H = Q * Y, C = Q * B, L = Q * O, E = X * Y, J = X * B, h = X * O;
  return K[0] = 1 - V - L, K[1] = G + h, K[2] = H - J, K[3] = 0, K[4] = G - h, K[5] = 1 - Z - L, K[6] = C + E, K[7] = 0, K[8] = H + J, K[9] = C - E, K[10] = 1 - Z - V, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var MK = function(K, W, $, N, Q, X, Y) {
  var B = 1 / ($ - W), O = 1 / (Q - N), Z = 1 / (X - Y);
  return K[0] = X * 2 * B, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = X * 2 * O, K[6] = 0, K[7] = 0, K[8] = ($ + W) * B, K[9] = (Q + N) * O, K[10] = (Y + X) * Z, K[11] = -1, K[12] = 0, K[13] = 0, K[14] = Y * X * 2 * Z, K[15] = 0, K;
};
var V0 = function(K, W, $, N, Q) {
  var X = 1 / Math.tan(W / 2), Y;
  if (K[0] = X / $, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = X, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = -1, K[12] = 0, K[13] = 0, K[15] = 0, Q != null && Q !== Infinity)
    Y = 1 / (N - Q), K[10] = (Q + N) * Y, K[14] = 2 * Q * N * Y;
  else
    K[10] = -1, K[14] = -2 * N;
  return K;
};
var pK = function(K, W, $, N, Q) {
  var X = 1 / Math.tan(W / 2), Y;
  if (K[0] = X / $, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = X, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = -1, K[12] = 0, K[13] = 0, K[15] = 0, Q != null && Q !== Infinity)
    Y = 1 / (N - Q), K[10] = Q * Y, K[14] = Q * N * Y;
  else
    K[10] = -1, K[14] = -N;
  return K;
};
var FK = function(K, W, $, N) {
  var Q = Math.tan(W.upDegrees * Math.PI / 180), X = Math.tan(W.downDegrees * Math.PI / 180), Y = Math.tan(W.leftDegrees * Math.PI / 180), B = Math.tan(W.rightDegrees * Math.PI / 180), O = 2 / (Y + B), Z = 2 / (Q + X);
  return K[0] = O, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = Z, K[6] = 0, K[7] = 0, K[8] = -((Y - B) * O * 0.5), K[9] = (Q - X) * Z * 0.5, K[10] = N / ($ - N), K[11] = -1, K[12] = 0, K[13] = 0, K[14] = N * $ / ($ - N), K[15] = 0, K;
};
var C0 = function(K, W, $, N, Q, X, Y) {
  var B = 1 / (W - $), O = 1 / (N - Q), Z = 1 / (X - Y);
  return K[0] = -2 * B, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = -2 * O, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 2 * Z, K[11] = 0, K[12] = (W + $) * B, K[13] = (Q + N) * O, K[14] = (Y + X) * Z, K[15] = 1, K;
};
var fK = function(K, W, $, N, Q, X, Y) {
  var B = 1 / (W - $), O = 1 / (N - Q), Z = 1 / (X - Y);
  return K[0] = -2 * B, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = -2 * O, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = Z, K[11] = 0, K[12] = (W + $) * B, K[13] = (Q + N) * O, K[14] = X * Z, K[15] = 1, K;
};
var wK = function(K, W, $, N) {
  var Q, X, Y, B, O, Z, G, V, H, C, L = W[0], E = W[1], J = W[2], h = N[0], k = N[1], A = N[2], P = $[0], I = $[1], U = $[2];
  if (Math.abs(L - P) < D && Math.abs(E - I) < D && Math.abs(J - U) < D)
    return O0(K);
  if (G = L - P, V = E - I, H = J - U, C = 1 / Math.hypot(G, V, H), G *= C, V *= C, H *= C, Q = k * H - A * V, X = A * G - h * H, Y = h * V - k * G, C = Math.hypot(Q, X, Y), !C)
    Q = 0, X = 0, Y = 0;
  else
    C = 1 / C, Q *= C, X *= C, Y *= C;
  if (B = V * Y - H * X, O = H * Q - G * Y, Z = G * X - V * Q, C = Math.hypot(B, O, Z), !C)
    B = 0, O = 0, Z = 0;
  else
    C = 1 / C, B *= C, O *= C, Z *= C;
  return K[0] = Q, K[1] = B, K[2] = G, K[3] = 0, K[4] = X, K[5] = O, K[6] = V, K[7] = 0, K[8] = Y, K[9] = Z, K[10] = H, K[11] = 0, K[12] = -(Q * L + X * E + Y * J), K[13] = -(B * L + O * E + Z * J), K[14] = -(G * L + V * E + H * J), K[15] = 1, K;
};
var vK = function(K, W, $, N) {
  var Q = W[0], X = W[1], Y = W[2], B = N[0], O = N[1], Z = N[2], G = Q - $[0], V = X - $[1], H = Y - $[2], C = G * G + V * V + H * H;
  if (C > 0)
    C = 1 / Math.sqrt(C), G *= C, V *= C, H *= C;
  var L = O * H - Z * V, E = Z * G - B * H, J = B * V - O * G;
  if (C = L * L + E * E + J * J, C > 0)
    C = 1 / Math.sqrt(C), L *= C, E *= C, J *= C;
  return K[0] = L, K[1] = E, K[2] = J, K[3] = 0, K[4] = V * J - H * E, K[5] = H * L - G * J, K[6] = G * E - V * L, K[7] = 0, K[8] = G, K[9] = V, K[10] = H, K[11] = 0, K[12] = Q, K[13] = X, K[14] = Y, K[15] = 1, K;
};
var dK = function(K) {
  return "mat4(" + K[0] + ", " + K[1] + ", " + K[2] + ", " + K[3] + ", " + K[4] + ", " + K[5] + ", " + K[6] + ", " + K[7] + ", " + K[8] + ", " + K[9] + ", " + K[10] + ", " + K[11] + ", " + K[12] + ", " + K[13] + ", " + K[14] + ", " + K[15] + ")";
};
var nK = function(K) {
  return Math.hypot(K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8], K[9], K[10], K[11], K[12], K[13], K[14], K[15]);
};
var iK = function(K, W, $) {
  return K[0] = W[0] + $[0], K[1] = W[1] + $[1], K[2] = W[2] + $[2], K[3] = W[3] + $[3], K[4] = W[4] + $[4], K[5] = W[5] + $[5], K[6] = W[6] + $[6], K[7] = W[7] + $[7], K[8] = W[8] + $[8], K[9] = W[9] + $[9], K[10] = W[10] + $[10], K[11] = W[11] + $[11], K[12] = W[12] + $[12], K[13] = W[13] + $[13], K[14] = W[14] + $[14], K[15] = W[15] + $[15], K;
};
var H0 = function(K, W, $) {
  return K[0] = W[0] - $[0], K[1] = W[1] - $[1], K[2] = W[2] - $[2], K[3] = W[3] - $[3], K[4] = W[4] - $[4], K[5] = W[5] - $[5], K[6] = W[6] - $[6], K[7] = W[7] - $[7], K[8] = W[8] - $[8], K[9] = W[9] - $[9], K[10] = W[10] - $[10], K[11] = W[11] - $[11], K[12] = W[12] - $[12], K[13] = W[13] - $[13], K[14] = W[14] - $[14], K[15] = W[15] - $[15], K;
};
var lK = function(K, W, $) {
  return K[0] = W[0] * $, K[1] = W[1] * $, K[2] = W[2] * $, K[3] = W[3] * $, K[4] = W[4] * $, K[5] = W[5] * $, K[6] = W[6] * $, K[7] = W[7] * $, K[8] = W[8] * $, K[9] = W[9] * $, K[10] = W[10] * $, K[11] = W[11] * $, K[12] = W[12] * $, K[13] = W[13] * $, K[14] = W[14] * $, K[15] = W[15] * $, K;
};
var cK = function(K, W, $, N) {
  return K[0] = W[0] + $[0] * N, K[1] = W[1] + $[1] * N, K[2] = W[2] + $[2] * N, K[3] = W[3] + $[3] * N, K[4] = W[4] + $[4] * N, K[5] = W[5] + $[5] * N, K[6] = W[6] + $[6] * N, K[7] = W[7] + $[7] * N, K[8] = W[8] + $[8] * N, K[9] = W[9] + $[9] * N, K[10] = W[10] + $[10] * N, K[11] = W[11] + $[11] * N, K[12] = W[12] + $[12] * N, K[13] = W[13] + $[13] * N, K[14] = W[14] + $[14] * N, K[15] = W[15] + $[15] * N, K;
};
var rK = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2] && K[3] === W[3] && K[4] === W[4] && K[5] === W[5] && K[6] === W[6] && K[7] === W[7] && K[8] === W[8] && K[9] === W[9] && K[10] === W[10] && K[11] === W[11] && K[12] === W[12] && K[13] === W[13] && K[14] === W[14] && K[15] === W[15];
};
var yK = function(K, W) {
  var $ = K[0], N = K[1], Q = K[2], X = K[3], Y = K[4], B = K[5], O = K[6], Z = K[7], G = K[8], V = K[9], H = K[10], C = K[11], L = K[12], E = K[13], J = K[14], h = K[15], k = W[0], A = W[1], P = W[2], I = W[3], U = W[4], g = W[5], _ = W[6], j = W[7], M = W[8], p = W[9], F = W[10], q = W[11], T = W[12], f = W[13], i = W[14], l = W[15];
  return Math.abs($ - k) <= D * Math.max(1, Math.abs($), Math.abs(k)) && Math.abs(N - A) <= D * Math.max(1, Math.abs(N), Math.abs(A)) && Math.abs(Q - P) <= D * Math.max(1, Math.abs(Q), Math.abs(P)) && Math.abs(X - I) <= D * Math.max(1, Math.abs(X), Math.abs(I)) && Math.abs(Y - U) <= D * Math.max(1, Math.abs(Y), Math.abs(U)) && Math.abs(B - g) <= D * Math.max(1, Math.abs(B), Math.abs(g)) && Math.abs(O - _) <= D * Math.max(1, Math.abs(O), Math.abs(_)) && Math.abs(Z - j) <= D * Math.max(1, Math.abs(Z), Math.abs(j)) && Math.abs(G - M) <= D * Math.max(1, Math.abs(G), Math.abs(M)) && Math.abs(V - p) <= D * Math.max(1, Math.abs(V), Math.abs(p)) && Math.abs(H - F) <= D * Math.max(1, Math.abs(H), Math.abs(F)) && Math.abs(C - q) <= D * Math.max(1, Math.abs(C), Math.abs(q)) && Math.abs(L - T) <= D * Math.max(1, Math.abs(L), Math.abs(T)) && Math.abs(E - f) <= D * Math.max(1, Math.abs(E), Math.abs(f)) && Math.abs(J - i) <= D * Math.max(1, Math.abs(J), Math.abs(i)) && Math.abs(h - l) <= D * Math.max(1, Math.abs(h), Math.abs(l));
};
var m = function() {
  var K = new S(3);
  if (S != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0;
  return K;
};
var mK = function(K) {
  var W = new S(3);
  return W[0] = K[0], W[1] = K[1], W[2] = K[2], W;
};
var E0 = function(K) {
  var W = K[0], $ = K[1], N = K[2];
  return Math.hypot(W, $, N);
};
var x = function(K, W, $) {
  var N = new S(3);
  return N[0] = K, N[1] = W, N[2] = $, N;
};
var xK = function(K, W) {
  return K[0] = W[0], K[1] = W[1], K[2] = W[2], K;
};
var eK = function(K, W, $, N) {
  return K[0] = W, K[1] = $, K[2] = N, K;
};
var bK = function(K, W, $) {
  return K[0] = W[0] + $[0], K[1] = W[1] + $[1], K[2] = W[2] + $[2], K;
};
var L0 = function(K, W, $) {
  return K[0] = W[0] - $[0], K[1] = W[1] - $[1], K[2] = W[2] - $[2], K;
};
var h0 = function(K, W, $) {
  return K[0] = W[0] * $[0], K[1] = W[1] * $[1], K[2] = W[2] * $[2], K;
};
var I0 = function(K, W, $) {
  return K[0] = W[0] / $[0], K[1] = W[1] / $[1], K[2] = W[2] / $[2], K;
};
var uK = function(K, W) {
  return K[0] = Math.ceil(W[0]), K[1] = Math.ceil(W[1]), K[2] = Math.ceil(W[2]), K;
};
var oK = function(K, W) {
  return K[0] = Math.floor(W[0]), K[1] = Math.floor(W[1]), K[2] = Math.floor(W[2]), K;
};
var tK = function(K, W, $) {
  return K[0] = Math.min(W[0], $[0]), K[1] = Math.min(W[1], $[1]), K[2] = Math.min(W[2], $[2]), K;
};
var aK = function(K, W, $) {
  return K[0] = Math.max(W[0], $[0]), K[1] = Math.max(W[1], $[1]), K[2] = Math.max(W[2], $[2]), K;
};
var KW = function(K, W) {
  return K[0] = Math.round(W[0]), K[1] = Math.round(W[1]), K[2] = Math.round(W[2]), K;
};
var WW = function(K, W, $) {
  return K[0] = W[0] * $, K[1] = W[1] * $, K[2] = W[2] * $, K;
};
var $W = function(K, W, $, N) {
  return K[0] = W[0] + $[0] * N, K[1] = W[1] + $[1] * N, K[2] = W[2] + $[2] * N, K;
};
var U0 = function(K, W) {
  var $ = W[0] - K[0], N = W[1] - K[1], Q = W[2] - K[2];
  return Math.hypot($, N, Q);
};
var P0 = function(K, W) {
  var $ = W[0] - K[0], N = W[1] - K[1], Q = W[2] - K[2];
  return $ * $ + N * N + Q * Q;
};
var A0 = function(K) {
  var W = K[0], $ = K[1], N = K[2];
  return W * W + $ * $ + N * N;
};
var NW = function(K, W) {
  return K[0] = -W[0], K[1] = -W[1], K[2] = -W[2], K;
};
var QW = function(K, W) {
  return K[0] = 1 / W[0], K[1] = 1 / W[1], K[2] = 1 / W[2], K;
};
var a = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = $ * $ + N * N + Q * Q;
  if (X > 0)
    X = 1 / Math.sqrt(X);
  return K[0] = W[0] * X, K[1] = W[1] * X, K[2] = W[2] * X, K;
};
var e = function(K, W) {
  return K[0] * W[0] + K[1] * W[1] + K[2] * W[2];
};
var r = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2], Y = $[0], B = $[1], O = $[2];
  return K[0] = Q * O - X * B, K[1] = X * Y - N * O, K[2] = N * B - Q * Y, K;
};
var XW = function(K, W, $, N) {
  var Q = W[0], X = W[1], Y = W[2];
  return K[0] = Q + N * ($[0] - Q), K[1] = X + N * ($[1] - X), K[2] = Y + N * ($[2] - Y), K;
};
var YW = function(K, W, $, N, Q, X) {
  var Y = X * X, B = Y * (2 * X - 3) + 1, O = Y * (X - 2) + X, Z = Y * (X - 1), G = Y * (3 - 2 * X);
  return K[0] = W[0] * B + $[0] * O + N[0] * Z + Q[0] * G, K[1] = W[1] * B + $[1] * O + N[1] * Z + Q[1] * G, K[2] = W[2] * B + $[2] * O + N[2] * Z + Q[2] * G, K;
};
var BW = function(K, W, $, N, Q, X) {
  var Y = 1 - X, B = Y * Y, O = X * X, Z = B * Y, G = 3 * X * B, V = 3 * O * Y, H = O * X;
  return K[0] = W[0] * Z + $[0] * G + N[0] * V + Q[0] * H, K[1] = W[1] * Z + $[1] * G + N[1] * V + Q[1] * H, K[2] = W[2] * Z + $[2] * G + N[2] * V + Q[2] * H, K;
};
var OW = function(K, W) {
  W = W || 1;
  var $ = d() * 2 * Math.PI, N = d() * 2 - 1, Q = Math.sqrt(1 - N * N) * W;
  return K[0] = Math.cos($) * Q, K[1] = Math.sin($) * Q, K[2] = N * W, K;
};
var ZW = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2], Y = $[3] * N + $[7] * Q + $[11] * X + $[15];
  return Y = Y || 1, K[0] = ($[0] * N + $[4] * Q + $[8] * X + $[12]) / Y, K[1] = ($[1] * N + $[5] * Q + $[9] * X + $[13]) / Y, K[2] = ($[2] * N + $[6] * Q + $[10] * X + $[14]) / Y, K;
};
var GW = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2];
  return K[0] = N * $[0] + Q * $[3] + X * $[6], K[1] = N * $[1] + Q * $[4] + X * $[7], K[2] = N * $[2] + Q * $[5] + X * $[8], K;
};
var JW = function(K, W, $) {
  var N = $[0], Q = $[1], X = $[2], Y = $[3], B = W[0], O = W[1], Z = W[2], G = Q * Z - X * O, V = X * B - N * Z, H = N * O - Q * B, C = Q * H - X * V, L = X * G - N * H, E = N * V - Q * G, J = Y * 2;
  return G *= J, V *= J, H *= J, C *= 2, L *= 2, E *= 2, K[0] = B + G + C, K[1] = O + V + L, K[2] = Z + H + E, K;
};
var VW = function(K, W, $, N) {
  var Q = [], X = [];
  return Q[0] = W[0] - $[0], Q[1] = W[1] - $[1], Q[2] = W[2] - $[2], X[0] = Q[0], X[1] = Q[1] * Math.cos(N) - Q[2] * Math.sin(N), X[2] = Q[1] * Math.sin(N) + Q[2] * Math.cos(N), K[0] = X[0] + $[0], K[1] = X[1] + $[1], K[2] = X[2] + $[2], K;
};
var CW = function(K, W, $, N) {
  var Q = [], X = [];
  return Q[0] = W[0] - $[0], Q[1] = W[1] - $[1], Q[2] = W[2] - $[2], X[0] = Q[2] * Math.sin(N) + Q[0] * Math.cos(N), X[1] = Q[1], X[2] = Q[2] * Math.cos(N) - Q[0] * Math.sin(N), K[0] = X[0] + $[0], K[1] = X[1] + $[1], K[2] = X[2] + $[2], K;
};
var HW = function(K, W, $, N) {
  var Q = [], X = [];
  return Q[0] = W[0] - $[0], Q[1] = W[1] - $[1], Q[2] = W[2] - $[2], X[0] = Q[0] * Math.cos(N) - Q[1] * Math.sin(N), X[1] = Q[0] * Math.sin(N) + Q[1] * Math.cos(N), X[2] = Q[2], K[0] = X[0] + $[0], K[1] = X[1] + $[1], K[2] = X[2] + $[2], K;
};
var EW = function(K, W) {
  var $ = K[0], N = K[1], Q = K[2], X = W[0], Y = W[1], B = W[2], O = Math.sqrt($ * $ + N * N + Q * Q), Z = Math.sqrt(X * X + Y * Y + B * B), G = O * Z, V = G && e(K, W) / G;
  return Math.acos(Math.min(Math.max(V, -1), 1));
};
var LW = function(K) {
  return K[0] = 0, K[1] = 0, K[2] = 0, K;
};
var hW = function(K) {
  return "vec3(" + K[0] + ", " + K[1] + ", " + K[2] + ")";
};
var IW = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2];
};
var UW = function(K, W) {
  var $ = K[0], N = K[1], Q = K[2], X = W[0], Y = W[1], B = W[2];
  return Math.abs($ - X) <= D * Math.max(1, Math.abs($), Math.abs(X)) && Math.abs(N - Y) <= D * Math.max(1, Math.abs(N), Math.abs(Y)) && Math.abs(Q - B) <= D * Math.max(1, Math.abs(Q), Math.abs(B));
};
var _W = function() {
  var K = new S(4);
  if (S != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0, K[3] = 0;
  return K;
};
var D0 = function(K) {
  var W = new S(4);
  return W[0] = K[0], W[1] = K[1], W[2] = K[2], W[3] = K[3], W;
};
var k0 = function(K, W, $, N) {
  var Q = new S(4);
  return Q[0] = K, Q[1] = W, Q[2] = $, Q[3] = N, Q;
};
var T0 = function(K, W) {
  return K[0] = W[0], K[1] = W[1], K[2] = W[2], K[3] = W[3], K;
};
var R0 = function(K, W, $, N, Q) {
  return K[0] = W, K[1] = $, K[2] = N, K[3] = Q, K;
};
var S0 = function(K, W, $) {
  return K[0] = W[0] + $[0], K[1] = W[1] + $[1], K[2] = W[2] + $[2], K[3] = W[3] + $[3], K;
};
var _0 = function(K, W, $) {
  return K[0] = W[0] * $, K[1] = W[1] * $, K[2] = W[2] * $, K[3] = W[3] * $, K;
};
var j0 = function(K) {
  var W = K[0], $ = K[1], N = K[2], Q = K[3];
  return Math.hypot(W, $, N, Q);
};
var M0 = function(K) {
  var W = K[0], $ = K[1], N = K[2], Q = K[3];
  return W * W + $ * $ + N * N + Q * Q;
};
var g0 = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = $ * $ + N * N + Q * Q + X * X;
  if (Y > 0)
    Y = 1 / Math.sqrt(Y);
  return K[0] = $ * Y, K[1] = N * Y, K[2] = Q * Y, K[3] = X * Y, K;
};
var p0 = function(K, W) {
  return K[0] * W[0] + K[1] * W[1] + K[2] * W[2] + K[3] * W[3];
};
var F0 = function(K, W, $, N) {
  var Q = W[0], X = W[1], Y = W[2], B = W[3];
  return K[0] = Q + N * ($[0] - Q), K[1] = X + N * ($[1] - X), K[2] = Y + N * ($[2] - Y), K[3] = B + N * ($[3] - B), K;
};
var q0 = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2] && K[3] === W[3];
};
var f0 = function(K, W) {
  var $ = K[0], N = K[1], Q = K[2], X = K[3], Y = W[0], B = W[1], O = W[2], Z = W[3];
  return Math.abs($ - Y) <= D * Math.max(1, Math.abs($), Math.abs(Y)) && Math.abs(N - B) <= D * Math.max(1, Math.abs(N), Math.abs(B)) && Math.abs(Q - O) <= D * Math.max(1, Math.abs(Q), Math.abs(O)) && Math.abs(X - Z) <= D * Math.max(1, Math.abs(X), Math.abs(Z));
};
var W0 = function() {
  var K = new S(4);
  if (S != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0;
  return K[3] = 1, K;
};
var MW = function(K) {
  return K[0] = 0, K[1] = 0, K[2] = 0, K[3] = 1, K;
};
var w0 = function(K, W, $) {
  $ = $ * 0.5;
  var N = Math.sin($);
  return K[0] = N * W[0], K[1] = N * W[1], K[2] = N * W[2], K[3] = Math.cos($), K;
};
var gW = function(K, W) {
  var $ = Math.acos(W[3]) * 2, N = Math.sin($ / 2);
  if (N > D)
    K[0] = W[0] / N, K[1] = W[1] / N, K[2] = W[2] / N;
  else
    K[0] = 1, K[1] = 0, K[2] = 0;
  return $;
};
var pW = function(K, W) {
  var $ = c0(K, W);
  return Math.acos(2 * $ * $ - 1);
};
var v0 = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2], Y = W[3], B = $[0], O = $[1], Z = $[2], G = $[3];
  return K[0] = N * G + Y * B + Q * Z - X * O, K[1] = Q * G + Y * O + X * B - N * Z, K[2] = X * G + Y * Z + N * O - Q * B, K[3] = Y * G - N * B - Q * O - X * Z, K;
};
var FW = function(K, W, $) {
  $ *= 0.5;
  var N = W[0], Q = W[1], X = W[2], Y = W[3], B = Math.sin($), O = Math.cos($);
  return K[0] = N * O + Y * B, K[1] = Q * O + X * B, K[2] = X * O - Q * B, K[3] = Y * O - N * B, K;
};
var qW = function(K, W, $) {
  $ *= 0.5;
  var N = W[0], Q = W[1], X = W[2], Y = W[3], B = Math.sin($), O = Math.cos($);
  return K[0] = N * O - X * B, K[1] = Q * O + Y * B, K[2] = X * O + N * B, K[3] = Y * O - Q * B, K;
};
var fW = function(K, W, $) {
  $ *= 0.5;
  var N = W[0], Q = W[1], X = W[2], Y = W[3], B = Math.sin($), O = Math.cos($);
  return K[0] = N * O + Q * B, K[1] = Q * O - N * B, K[2] = X * O + Y * B, K[3] = Y * O - X * B, K;
};
var wW = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2];
  return K[0] = $, K[1] = N, K[2] = Q, K[3] = Math.sqrt(Math.abs(1 - $ * $ - N * N - Q * Q)), K;
};
var d0 = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = Math.sqrt($ * $ + N * N + Q * Q), B = Math.exp(X), O = Y > 0 ? B * Math.sin(Y) / Y : 0;
  return K[0] = $ * O, K[1] = N * O, K[2] = Q * O, K[3] = B * Math.cos(Y), K;
};
var n0 = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = Math.sqrt($ * $ + N * N + Q * Q), B = Y > 0 ? Math.atan2(Y, X) / Y : 0;
  return K[0] = $ * B, K[1] = N * B, K[2] = Q * B, K[3] = 0.5 * Math.log($ * $ + N * N + Q * Q + X * X), K;
};
var vW = function(K, W, $) {
  return n0(K, W), l0(K, K, $), d0(K, K), K;
};
var b = function(K, W, $, N) {
  var Q = W[0], X = W[1], Y = W[2], B = W[3], O = $[0], Z = $[1], G = $[2], V = $[3], H, C, L, E, J;
  if (C = Q * O + X * Z + Y * G + B * V, C < 0)
    C = -C, O = -O, Z = -Z, G = -G, V = -V;
  if (1 - C > D)
    H = Math.acos(C), L = Math.sin(H), E = Math.sin((1 - N) * H) / L, J = Math.sin(N * H) / L;
  else
    E = 1 - N, J = N;
  return K[0] = E * Q + J * O, K[1] = E * X + J * Z, K[2] = E * Y + J * G, K[3] = E * B + J * V, K;
};
var dW = function(K) {
  var W = d(), $ = d(), N = d(), Q = Math.sqrt(1 - W), X = Math.sqrt(W);
  return K[0] = Q * Math.sin(2 * Math.PI * $), K[1] = Q * Math.cos(2 * Math.PI * $), K[2] = X * Math.sin(2 * Math.PI * N), K[3] = X * Math.cos(2 * Math.PI * N), K;
};
var nW = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = $ * $ + N * N + Q * Q + X * X, B = Y ? 1 / Y : 0;
  return K[0] = -$ * B, K[1] = -N * B, K[2] = -Q * B, K[3] = X * B, K;
};
var iW = function(K, W) {
  return K[0] = -W[0], K[1] = -W[1], K[2] = -W[2], K[3] = W[3], K;
};
var i0 = function(K, W) {
  var $ = W[0] + W[4] + W[8], N;
  if ($ > 0)
    N = Math.sqrt($ + 1), K[3] = 0.5 * N, N = 0.5 / N, K[0] = (W[5] - W[7]) * N, K[1] = (W[6] - W[2]) * N, K[2] = (W[1] - W[3]) * N;
  else {
    var Q = 0;
    if (W[4] > W[0])
      Q = 1;
    if (W[8] > W[Q * 3 + Q])
      Q = 2;
    var X = (Q + 1) % 3, Y = (Q + 2) % 3;
    N = Math.sqrt(W[Q * 3 + Q] - W[X * 3 + X] - W[Y * 3 + Y] + 1), K[Q] = 0.5 * N, N = 0.5 / N, K[3] = (W[X * 3 + Y] - W[Y * 3 + X]) * N, K[X] = (W[X * 3 + Q] + W[Q * 3 + X]) * N, K[Y] = (W[Y * 3 + Q] + W[Q * 3 + Y]) * N;
  }
  return K;
};
var lW = function(K, W, $, N) {
  var Q = 0.5 * Math.PI / 180;
  W *= Q, $ *= Q, N *= Q;
  var X = Math.sin(W), Y = Math.cos(W), B = Math.sin($), O = Math.cos($), Z = Math.sin(N), G = Math.cos(N);
  return K[0] = X * O * G - Y * B * Z, K[1] = Y * B * G + X * O * Z, K[2] = Y * O * Z - X * B * G, K[3] = Y * O * G + X * B * Z, K;
};
var cW = function(K) {
  return "quat(" + K[0] + ", " + K[1] + ", " + K[2] + ", " + K[3] + ")";
};
var a0 = Object.defineProperty;
var t = (K, W) => {
  for (var $ in W)
    a0(K, $, { get: W[$], enumerable: true, configurable: true, set: (N) => W[$] = () => N });
};
var D = 0.000001;
var S = typeof Float32Array !== "undefined" ? Float32Array : Array;
var d = Math.random;
var G$ = Math.PI / 180;
if (!Math.hypot)
  Math.hypot = function() {
    var K = 0, W = arguments.length;
    while (W--)
      K += arguments[W] * arguments[W];
    return Math.sqrt(K);
  };
var R = {};
t(R, { transpose: () => {
  {
    return BK;
  }
}, translate: () => {
  {
    return JK;
  }
}, targetTo: () => {
  {
    return vK;
  }
}, subtract: () => {
  {
    return H0;
  }
}, sub: () => {
  {
    return sK;
  }
}, str: () => {
  {
    return dK;
  }
}, set: () => {
  {
    return YK;
  }
}, scale: () => {
  {
    return VK;
  }
}, rotateZ: () => {
  {
    return LK;
  }
}, rotateY: () => {
  {
    return EK;
  }
}, rotateX: () => {
  {
    return HK;
  }
}, rotate: () => {
  {
    return CK;
  }
}, perspectiveZO: () => {
  {
    return pK;
  }
}, perspectiveNO: () => {
  {
    return V0;
  }
}, perspectiveFromFieldOfView: () => {
  {
    return FK;
  }
}, perspective: () => {
  {
    return gK;
  }
}, orthoZO: () => {
  {
    return fK;
  }
}, orthoNO: () => {
  {
    return C0;
  }
}, ortho: () => {
  {
    return qK;
  }
}, multiplyScalarAndAdd: () => {
  {
    return cK;
  }
}, multiplyScalar: () => {
  {
    return lK;
  }
}, multiply: () => {
  {
    return Z0;
  }
}, mul: () => {
  {
    return zK;
  }
}, lookAt: () => {
  {
    return wK;
  }
}, invert: () => {
  {
    return OK;
  }
}, identity: () => {
  {
    return O0;
  }
}, getTranslation: () => {
  {
    return TK;
  }
}, getScaling: () => {
  {
    return J0;
  }
}, getRotation: () => {
  {
    return RK;
  }
}, frustum: () => {
  {
    return MK;
  }
}, fromZRotation: () => {
  {
    return DK;
  }
}, fromYRotation: () => {
  {
    return AK;
  }
}, fromXRotation: () => {
  {
    return PK;
  }
}, fromValues: () => {
  {
    return XK;
  }
}, fromTranslation: () => {
  {
    return hK;
  }
}, fromScaling: () => {
  {
    return IK;
  }
}, fromRotationTranslationScaleOrigin: () => {
  {
    return _K;
  }
}, fromRotationTranslationScale: () => {
  {
    return SK;
  }
}, fromRotationTranslation: () => {
  {
    return G0;
  }
}, fromRotation: () => {
  {
    return UK;
  }
}, fromQuat2: () => {
  {
    return kK;
  }
}, fromQuat: () => {
  {
    return jK;
  }
}, frob: () => {
  {
    return nK;
  }
}, exactEquals: () => {
  {
    return rK;
  }
}, equals: () => {
  {
    return yK;
  }
}, determinant: () => {
  {
    return GK;
  }
}, create: () => {
  {
    return $K;
  }
}, copy: () => {
  {
    return QK;
  }
}, clone: () => {
  {
    return NK;
  }
}, adjoint: () => {
  {
    return ZK;
  }
}, add: () => {
  {
    return iK;
  }
} });
var gK = V0;
var qK = C0;
var zK = Z0;
var sK = H0;
var z = {};
t(z, { str: () => {
  {
    return cW;
  }
}, squaredLength: () => {
  {
    return y0;
  }
}, sqrLen: () => {
  {
    return uW;
  }
}, sqlerp: () => {
  {
    return K$;
  }
}, slerp: () => {
  {
    return b;
  }
}, setAxisAngle: () => {
  {
    return w0;
  }
}, setAxes: () => {
  {
    return W$;
  }
}, set: () => {
  {
    return sW;
  }
}, scale: () => {
  {
    return l0;
  }
}, rotationTo: () => {
  {
    return aW;
  }
}, rotateZ: () => {
  {
    return fW;
  }
}, rotateY: () => {
  {
    return qW;
  }
}, rotateX: () => {
  {
    return FW;
  }
}, random: () => {
  {
    return dW;
  }
}, pow: () => {
  {
    return vW;
  }
}, normalize: () => {
  {
    return $0;
  }
}, multiply: () => {
  {
    return v0;
  }
}, mul: () => {
  {
    return xW;
  }
}, ln: () => {
  {
    return n0;
  }
}, lerp: () => {
  {
    return eW;
  }
}, length: () => {
  {
    return r0;
  }
}, len: () => {
  {
    return bW;
  }
}, invert: () => {
  {
    return nW;
  }
}, identity: () => {
  {
    return MW;
  }
}, getAxisAngle: () => {
  {
    return gW;
  }
}, getAngle: () => {
  {
    return pW;
  }
}, fromValues: () => {
  {
    return yW;
  }
}, fromMat3: () => {
  {
    return i0;
  }
}, fromEuler: () => {
  {
    return lW;
  }
}, exp: () => {
  {
    return d0;
  }
}, exactEquals: () => {
  {
    return oW;
  }
}, equals: () => {
  {
    return tW;
  }
}, dot: () => {
  {
    return c0;
  }
}, create: () => {
  {
    return W0;
  }
}, copy: () => {
  {
    return zW;
  }
}, conjugate: () => {
  {
    return iW;
  }
}, clone: () => {
  {
    return rW;
  }
}, calculateW: () => {
  {
    return wW;
  }
}, add: () => {
  {
    return mW;
  }
} });
var y = {};
t(y, { zero: () => {
  {
    return LW;
  }
}, transformQuat: () => {
  {
    return JW;
  }
}, transformMat4: () => {
  {
    return ZW;
  }
}, transformMat3: () => {
  {
    return GW;
  }
}, subtract: () => {
  {
    return L0;
  }
}, sub: () => {
  {
    return PW;
  }
}, str: () => {
  {
    return hW;
  }
}, squaredLength: () => {
  {
    return A0;
  }
}, squaredDistance: () => {
  {
    return P0;
  }
}, sqrLen: () => {
  {
    return RW;
  }
}, sqrDist: () => {
  {
    return TW;
  }
}, set: () => {
  {
    return eK;
  }
}, scaleAndAdd: () => {
  {
    return $W;
  }
}, scale: () => {
  {
    return WW;
  }
}, round: () => {
  {
    return KW;
  }
}, rotateZ: () => {
  {
    return HW;
  }
}, rotateY: () => {
  {
    return CW;
  }
}, rotateX: () => {
  {
    return VW;
  }
}, random: () => {
  {
    return OW;
  }
}, normalize: () => {
  {
    return a;
  }
}, negate: () => {
  {
    return NW;
  }
}, multiply: () => {
  {
    return h0;
  }
}, mul: () => {
  {
    return AW;
  }
}, min: () => {
  {
    return tK;
  }
}, max: () => {
  {
    return aK;
  }
}, lerp: () => {
  {
    return XW;
  }
}, length: () => {
  {
    return E0;
  }
}, len: () => {
  {
    return K0;
  }
}, inverse: () => {
  {
    return QW;
  }
}, hermite: () => {
  {
    return YW;
  }
}, fromValues: () => {
  {
    return x;
  }
}, forEach: () => {
  {
    return SW;
  }
}, floor: () => {
  {
    return oK;
  }
}, exactEquals: () => {
  {
    return IW;
  }
}, equals: () => {
  {
    return UW;
  }
}, dot: () => {
  {
    return e;
  }
}, divide: () => {
  {
    return I0;
  }
}, div: () => {
  {
    return DW;
  }
}, distance: () => {
  {
    return U0;
  }
}, dist: () => {
  {
    return kW;
  }
}, cross: () => {
  {
    return r;
  }
}, create: () => {
  {
    return m;
  }
}, copy: () => {
  {
    return xK;
  }
}, clone: () => {
  {
    return mK;
  }
}, ceil: () => {
  {
    return uK;
  }
}, bezier: () => {
  {
    return BW;
  }
}, angle: () => {
  {
    return EW;
  }
}, add: () => {
  {
    return bK;
  }
} });
var PW = L0;
var AW = h0;
var DW = I0;
var kW = U0;
var TW = P0;
var K0 = E0;
var RW = A0;
var SW = function() {
  var K = m();
  return function(W, $, N, Q, X, Y) {
    var B, O;
    if (!$)
      $ = 3;
    if (!N)
      N = 0;
    if (Q)
      O = Math.min(Q * $ + N, W.length);
    else
      O = W.length;
    for (B = N;B < O; B += $)
      K[0] = W[B], K[1] = W[B + 1], K[2] = W[B + 2], X(K, K, Y), W[B] = K[0], W[B + 1] = K[1], W[B + 2] = K[2];
    return W;
  };
}();
var J$ = function() {
  var K = _W();
  return function(W, $, N, Q, X, Y) {
    var B, O;
    if (!$)
      $ = 4;
    if (!N)
      N = 0;
    if (Q)
      O = Math.min(Q * $ + N, W.length);
    else
      O = W.length;
    for (B = N;B < O; B += $)
      K[0] = W[B], K[1] = W[B + 1], K[2] = W[B + 2], K[3] = W[B + 3], X(K, K, Y), W[B] = K[0], W[B + 1] = K[1], W[B + 2] = K[2], W[B + 3] = K[3];
    return W;
  };
}();
var rW = D0;
var yW = k0;
var zW = T0;
var sW = R0;
var mW = S0;
var xW = v0;
var l0 = _0;
var c0 = p0;
var eW = F0;
var r0 = j0;
var bW = r0;
var y0 = M0;
var uW = y0;
var $0 = g0;
var oW = q0;
var tW = f0;
var aW = function() {
  var K = m(), W = x(1, 0, 0), $ = x(0, 1, 0);
  return function(N, Q, X) {
    var Y = e(Q, X);
    if (Y < -0.999999) {
      if (r(K, W, Q), K0(K) < 0.000001)
        r(K, $, Q);
      return a(K, K), w0(N, K, Math.PI), N;
    } else if (Y > 0.999999)
      return N[0] = 0, N[1] = 0, N[2] = 0, N[3] = 1, N;
    else
      return r(K, Q, X), N[0] = K[0], N[1] = K[1], N[2] = K[2], N[3] = 1 + Y, $0(N, N);
  };
}();
var K$ = function() {
  var K = W0(), W = W0();
  return function($, N, Q, X, Y, B) {
    return b(K, N, Y, B), b(W, Q, X, B), b($, K, W, 2 * B * (1 - B)), $;
  };
}();
var W$ = function() {
  var K = B0();
  return function(W, $, N, Q) {
    return K[0] = N[0], K[3] = N[1], K[6] = N[2], K[1] = Q[0], K[4] = Q[1], K[7] = Q[2], K[2] = -$[0], K[5] = -$[1], K[8] = -$[2], $0(W, i0(W, K));
  };
}();
var $$ = Math.PI / 90;
var N0 = [0, 0, 0];
var z0 = R.create();
var s0 = R.create();
var u = z.create();

class s {
  #K = Float32Array.from(R.create());
  static HIDDEN = s.create().scale(0, 0, 0);
  static IDENTITY = s.create();
  constructor() {
    this.identity();
  }
  static create() {
    return new s;
  }
  copy(K) {
    return R.copy(this.#K, K.getMatrix()), this;
  }
  identity() {
    return R.identity(this.#K), this;
  }
  invert(K) {
    return R.invert(this.#K, K?.getMatrix() ?? this.getMatrix()), this;
  }
  multiply(K) {
    return R.multiply(this.#K, this.#K, K.getMatrix()), this;
  }
  multiply2(K, W) {
    return R.multiply(this.#K, K.getMatrix(), W.getMatrix()), this;
  }
  multiply3(K, W, $) {
    return this.multiply2(K, W), this.multiply($), this;
  }
  translate(K, W, $) {
    const N = N0;
    return N[0] = K, N[1] = W, N[2] = $, this.move(N);
  }
  move(K) {
    return R.translate(this.#K, this.#K, K), this;
  }
  rotateX(K) {
    return R.rotateX(this.#K, this.#K, K), this;
  }
  rotateY(K) {
    return R.rotateY(this.#K, this.#K, K), this;
  }
  rotateZ(K) {
    return R.rotateZ(this.#K, this.#K, K), this;
  }
  setXRotation(K) {
    return R.fromXRotation(this.getMatrix(), K), this;
  }
  setYRotation(K) {
    return R.fromYRotation(this.getMatrix(), K), this;
  }
  scale(K, W, $) {
    return R.scale(this.#K, this.#K, [K, W ?? K, $ ?? K]), this;
  }
  perspective(K, W, $, N) {
    return R.perspective(this.#K, K * $$, W, $, N), this;
  }
  ortho(K, W, $, N, Q, X) {
    return R.ortho(this.#K, K, W, $, N, Q, X), this;
  }
  combine(K, W, $ = 0.5) {
    return R.multiplyScalar(z0, K.getMatrix(), 1 - $), R.multiplyScalar(s0, W.getMatrix(), $), R.add(this.#K, z0, s0), this;
  }
  static getMoveVector(K, W, $, N) {
    const Q = N0;
    if (Q[0] = K, Q[1] = W, Q[2] = $, N)
      R.getRotation(u, N.getMatrix()), z.invert(u, u), y.transformQuat(Q, Q, u);
    return Q;
  }
  getPosition() {
    const K = N0;
    return K[0] = this.#K[12], K[1] = this.#K[13], K[2] = this.#K[14], K;
  }
  setVector(K) {
    return this.setPosition(K[0], K[1], K[2]);
  }
  setPosition(K, W, $) {
    return this.#K[12] = K, this.#K[13] = W, this.#K[14] = $, this;
  }
  getMatrix() {
    return this.#K;
  }
}
var w = s;

class m0 {
  w;
  q;
  #K;
  #W = false;
  #$ = 0;
  #N;
  #Q;
  constructor(K, W, $) {
    this.getValue = W, this.apply = $, this.#Q = K, this.#K = this.getValue(K);
  }
  set element(K) {
    this.#Q = K, this.#K = this.getValue(K), this.#N = undefined;
  }
  setGoal(K, W, $) {
    if (this.#N && this.#N !== $)
      return;
    if (this.#K !== K || this.#$ !== W)
      this.#$ = W, this.#K = K, this.#N = $, this.#W = true;
  }
  get goal() {
    return this.#K;
  }
  update(K) {
    if (this.#W) {
      const W = this.getValue(this.#Q), $ = this.goal - W, N = Math.min(Math.abs($), this.#$ * K);
      if (N <= 0.01)
        this.apply(this.#Q, this.goal), this.#W = false, this.#N = undefined;
      else
        this.apply(this.#Q, W + N * Math.sign($));
    }
    return this.#W;
  }
}

class x0 {
  i;
  f;
  warningLimit = 50000;
  #K = new Set;
  #W = [];
  constructor(K, W) {
    this.initCall = K, this.onRecycle = W;
  }
  create(...K) {
    const W = this.#W.pop();
    if (W)
      return this.#K.add(W), this.initCall(W, ...K);
    const $ = this.initCall(undefined, ...K);
    return this.#K.add($), this.#N(), $;
  }
  recycle(K) {
    this.#K.delete(K), this.#$(K);
  }
  recycleAll() {
    for (let K of this.#K)
      this.#$(K);
    this.#K.clear();
  }
  clear() {
    this.#W.length = 0, this.#K.clear();
  }
  countObjectsInExistence() {
    return this.#K.size + this.#W.length;
  }
  #$(K) {
    this.#W.push(K), this.onRecycle?.(K);
  }
  #N() {
    if (this.countObjectsInExistence() === this.warningLimit)
      console.warn("ObjectPool already created", this.#K.size + this.#W.length, "in", this.constructor.name);
  }
}

class e0 extends x0 {
  constructor() {
    super((K, W) => {
      if (!K)
        return new m0(W, ($) => $.valueOf(), ($, N) => $.setValue(N));
      return K.element = W, K;
    });
  }
}
var N$ = new e0;

class n {
  w;
  q;
  #K = 0;
  #W;
  constructor(K = 0, W, $ = N$) {
    this.onChange = W, this.pool = $, this.#K = K;
  }
  valueOf() {
    return this.#K;
  }
  setValue(K) {
    if (K !== this.#K)
      this.#K = K, this.onChange?.(this.#K);
    return this;
  }
  addValue(K) {
    return this.setValue(this.#K + K), this;
  }
  update(K) {
    if (this.#W) {
      const W = !!this.#W.update(K);
      if (!W)
        this.pool.recycle(this.#W), this.#W = undefined;
      return W;
    }
    return false;
  }
  refresh({ deltaTime: K, stopUpdate: W }) {
    if (!this.update(K))
      W();
  }
  progressTowards(K, W, $, N) {
    if (!this.#W)
      this.#W = this.pool.create(this);
    if (this.#W.setGoal(K, W, $), N)
      N.loop(this, undefined);
  }
  get goal() {
    return this.#W?.goal ?? this.valueOf();
  }
}
class u0 {
  i;
  f;
  warningLimit = 50000;
  #K = new Set;
  #W = [];
  constructor(K, W) {
    this.initCall = K, this.onRecycle = W;
  }
  create(...K) {
    const W = this.#W.pop();
    if (W)
      return this.#K.add(W), this.initCall(W, ...K);
    const $ = this.initCall(undefined, ...K);
    return this.#K.add($), this.#N(), $;
  }
  recycle(K) {
    this.#K.delete(K), this.#$(K);
  }
  recycleAll() {
    for (let K of this.#K)
      this.#$(K);
    this.#K.clear();
  }
  clear() {
    this.#W.length = 0, this.#K.clear();
  }
  countObjectsInExistence() {
    return this.#K.size + this.#W.length;
  }
  #$(K) {
    this.#W.push(K), this.onRecycle?.(K);
  }
  #N() {
    if (this.countObjectsInExistence() === this.warningLimit)
      console.warn("ObjectPool already created", this.#K.size + this.#W.length, "in", this.constructor.name);
  }
}

class Q0 extends u0 {
  constructor() {
    super((K, W, $, N) => {
      if (!K)
        return [W, $, N];
      return K[0] = W, K[1] = $, K[2] = N, K;
    });
  }
}

class o {
  vectorPool;
  motor;
  data;
  constructor({ motor: K, vectorPool: W }) {
    this.motor = K, this.vectorPool = W ?? new Q0, this.data = { vectorPool: this.vectorPool };
  }
  refresh({ data: { vectorPool: K } }) {
    K.recycleAll();
  }
  transformToPosition(K) {
    this.motor.scheduleUpdate(this, this.data);
    const W = K.getMatrix();
    return this.vectorPool.create(W[12], W[13], W[14]);
  }
  toVector(K, W, $) {
    return this.motor.scheduleUpdate(this, this.data), this.vectorPool.create(K, W, $);
  }
  static transformToPosition(K, W) {
    const $ = K.getMatrix();
    W[0] = $[12], W[1] = $[13], W[2] = $[14];
  }
}
var v;
(function(N) {
  N[N["AT_POSITION"] = 0] = "AT_POSITION";
  N[N["MOVED"] = 1] = "MOVED";
  N[N["BLOCKED"] = 2] = "BLOCKED";
})(v || (v = {}));

class o0 {
  #K = w.create().setPosition(0, 0, 0);
  #W = new Set;
  #$;
  position = [0, 0, 0];
  moveBlocker;
  constructor({ positionUtils: K, blocker: W }, $) {
    if (this.#$ = K, this.moveBlocker = W, $)
      this.onChange($);
  }
  onChange(K) {
    return this.#W.add(K), this;
  }
  removeChangeListener(K) {
    this.#W.delete(K);
  }
  changedPosition(K, W, $) {
    o.transformToPosition(this.#K, this.position);
    for (let N of this.#W)
      N(K, W, $);
  }
  moveBy(K, W, $, N) {
    const Q = w.getMoveVector(K, W, $, N), X = this.moveBlocker?.isBlocked(this.#$.toVector(this.position[0] + Q[0], this.position[1] + Q[1], this.position[2] + Q[2]), this.position);
    if (!X)
      if (Q[0] || Q[1] || Q[2])
        this.#K.move(Q), this.changedPosition(K, W, $);
      else
        return v.AT_POSITION;
    return X ? v.BLOCKED : v.MOVED;
  }
  moveTo(K, W, $) {
    if (this.position[0] === K && this.position[1] === W && this.position[2] === $)
      return v.AT_POSITION;
    const N = this.moveBlocker?.isBlocked(this.#$.toVector(K, W, $), this.position);
    if (!N) {
      const [Q, X, Y] = this.#K.getPosition();
      if (Q !== K || X !== W || Y !== $) {
        const B = K - Q, O = W - X, Z = $ - Y;
        this.#K.setPosition(K, W, $), this.changedPosition(B, O, Z);
      }
    }
    return N ? v.BLOCKED : v.MOVED;
  }
  movedTo(K, W, $) {
    return this.moveTo(K, W, $), this;
  }
  gotoPos(K, W, $, N = 0.1) {
    const Q = this.position, X = K - Q[0], Y = W - Q[1], B = $ - Q[2], O = Math.sqrt(X * X + Y * Y + B * B);
    if (O > 0.01) {
      const Z = Math.min(O, N);
      return this.moveBy(X / O * Z, Y / O * Z, B / O * Z);
    } else
      return this.moveTo(K, W, $);
  }
  getMatrix() {
    return this.#K.getMatrix();
  }
}
var Y$ = 1;
var B$ = 1;

class t0 {
  K;
  #K = w.create();
  #W = w.create();
  #$ = w.create();
  #N = [0, 0];
  perspective;
  zoom;
  constructor(K) {
    this.onChange = K;
    this.perspective = new n(Y$, K), this.zoom = new n(B$, (W) => {
      this.configure(this.#N, W);
    });
  }
  configPerspectiveMatrix(K, W, $, N) {
    this.#W.perspective(K, W, $, N);
  }
  configOrthoMatrix(K, W, $, N) {
    this.#$.ortho(-K / 2, K / 2, -W / 2, W / 2, $, N);
  }
  configure(K, W, $ = 0.5, N = 1e4) {
    if (!W)
      W = this.zoom.valueOf();
    this.#N[0] = K[0], this.#N[1] = K[1];
    const Q = this.#N[0] / this.#N[1], X = 45 / Math.sqrt(W);
    this.configPerspectiveMatrix(X, Q, Math.max($, 0.00001), N), this.configOrthoMatrix(Q / W / W, 1 / W / W, -N, N), this.onChange?.();
  }
  getMatrix() {
    return this.#K.combine(this.#$, this.#W, this.perspective.valueOf()), this.#K.getMatrix();
  }
}
export {
  w as Matrix
};
