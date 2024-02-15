// /Users/vincent/dok-matrix/example/node_modules/dok-matrix/dist/index.js
var Z0 = function() {
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
var XK = function(K, W, $, N, Q, X, Y, Z, B, O, G, C, V, H, L, E) {
  var J = new S(16);
  return J[0] = K, J[1] = W, J[2] = $, J[3] = N, J[4] = Q, J[5] = X, J[6] = Y, J[7] = Z, J[8] = B, J[9] = O, J[10] = G, J[11] = C, J[12] = V, J[13] = H, J[14] = L, J[15] = E, J;
};
var YK = function(K, W, $, N, Q, X, Y, Z, B, O, G, C, V, H, L, E, J) {
  return K[0] = W, K[1] = $, K[2] = N, K[3] = Q, K[4] = X, K[5] = Y, K[6] = Z, K[7] = B, K[8] = O, K[9] = G, K[10] = C, K[11] = V, K[12] = H, K[13] = L, K[14] = E, K[15] = J, K;
};
var B0 = function(K) {
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var ZK = function(K, W) {
  if (K === W) {
    var $ = W[1], N = W[2], Q = W[3], X = W[6], Y = W[7], Z = W[11];
    K[1] = W[4], K[2] = W[8], K[3] = W[12], K[4] = $, K[6] = W[9], K[7] = W[13], K[8] = N, K[9] = X, K[11] = W[14], K[12] = Q, K[13] = Y, K[14] = Z;
  } else
    K[0] = W[0], K[1] = W[4], K[2] = W[8], K[3] = W[12], K[4] = W[1], K[5] = W[5], K[6] = W[9], K[7] = W[13], K[8] = W[2], K[9] = W[6], K[10] = W[10], K[11] = W[14], K[12] = W[3], K[13] = W[7], K[14] = W[11], K[15] = W[15];
  return K;
};
var BK = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = W[4], Z = W[5], B = W[6], O = W[7], G = W[8], C = W[9], V = W[10], H = W[11], L = W[12], E = W[13], J = W[14], I = W[15], k = $ * Z - N * Y, A = $ * B - Q * Y, P = $ * O - X * Y, U = N * B - Q * Z, h = N * O - X * Z, g = Q * O - X * B, _ = G * E - C * L, j = G * J - V * L, M = G * I - H * L, p = C * J - V * E, F = C * I - H * E, q = V * I - H * J, R = k * q - A * F + P * p + U * M - h * j + g * _;
  if (!R)
    return null;
  return R = 1 / R, K[0] = (Z * q - B * F + O * p) * R, K[1] = (Q * F - N * q - X * p) * R, K[2] = (E * g - J * h + I * U) * R, K[3] = (V * h - C * g - H * U) * R, K[4] = (B * M - Y * q - O * j) * R, K[5] = ($ * q - Q * M + X * j) * R, K[6] = (J * P - L * g - I * A) * R, K[7] = (G * g - V * P + H * A) * R, K[8] = (Y * F - Z * M + O * _) * R, K[9] = (N * M - $ * F - X * _) * R, K[10] = (L * h - E * P + I * k) * R, K[11] = (C * P - G * h - H * k) * R, K[12] = (Z * j - Y * p - B * _) * R, K[13] = ($ * p - N * j + Q * _) * R, K[14] = (E * A - L * U - J * k) * R, K[15] = (G * U - C * A + V * k) * R, K;
};
var OK = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = W[4], Z = W[5], B = W[6], O = W[7], G = W[8], C = W[9], V = W[10], H = W[11], L = W[12], E = W[13], J = W[14], I = W[15];
  return K[0] = Z * (V * I - H * J) - C * (B * I - O * J) + E * (B * H - O * V), K[1] = -(N * (V * I - H * J) - C * (Q * I - X * J) + E * (Q * H - X * V)), K[2] = N * (B * I - O * J) - Z * (Q * I - X * J) + E * (Q * O - X * B), K[3] = -(N * (B * H - O * V) - Z * (Q * H - X * V) + C * (Q * O - X * B)), K[4] = -(Y * (V * I - H * J) - G * (B * I - O * J) + L * (B * H - O * V)), K[5] = $ * (V * I - H * J) - G * (Q * I - X * J) + L * (Q * H - X * V), K[6] = -($ * (B * I - O * J) - Y * (Q * I - X * J) + L * (Q * O - X * B)), K[7] = $ * (B * H - O * V) - Y * (Q * H - X * V) + G * (Q * O - X * B), K[8] = Y * (C * I - H * E) - G * (Z * I - O * E) + L * (Z * H - O * C), K[9] = -($ * (C * I - H * E) - G * (N * I - X * E) + L * (N * H - X * C)), K[10] = $ * (Z * I - O * E) - Y * (N * I - X * E) + L * (N * O - X * Z), K[11] = -($ * (Z * H - O * C) - Y * (N * H - X * C) + G * (N * O - X * Z)), K[12] = -(Y * (C * J - V * E) - G * (Z * J - B * E) + L * (Z * V - B * C)), K[13] = $ * (C * J - V * E) - G * (N * J - Q * E) + L * (N * V - Q * C), K[14] = -($ * (Z * J - B * E) - Y * (N * J - Q * E) + L * (N * B - Q * Z)), K[15] = $ * (Z * V - B * C) - Y * (N * V - Q * C) + G * (N * B - Q * Z), K;
};
var GK = function(K) {
  var W = K[0], $ = K[1], N = K[2], Q = K[3], X = K[4], Y = K[5], Z = K[6], B = K[7], O = K[8], G = K[9], C = K[10], V = K[11], H = K[12], L = K[13], E = K[14], J = K[15], I = W * Y - $ * X, k = W * Z - N * X, A = W * B - Q * X, P = $ * Z - N * Y, U = $ * B - Q * Y, h = N * B - Q * Z, g = O * L - G * H, _ = O * E - C * H, j = O * J - V * H, M = G * E - C * L, p = G * J - V * L, F = C * J - V * E;
  return I * F - k * p + A * M + P * j - U * _ + h * g;
};
var O0 = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2], Y = W[3], Z = W[4], B = W[5], O = W[6], G = W[7], C = W[8], V = W[9], H = W[10], L = W[11], E = W[12], J = W[13], I = W[14], k = W[15], A = $[0], P = $[1], U = $[2], h = $[3];
  return K[0] = A * N + P * Z + U * C + h * E, K[1] = A * Q + P * B + U * V + h * J, K[2] = A * X + P * O + U * H + h * I, K[3] = A * Y + P * G + U * L + h * k, A = $[4], P = $[5], U = $[6], h = $[7], K[4] = A * N + P * Z + U * C + h * E, K[5] = A * Q + P * B + U * V + h * J, K[6] = A * X + P * O + U * H + h * I, K[7] = A * Y + P * G + U * L + h * k, A = $[8], P = $[9], U = $[10], h = $[11], K[8] = A * N + P * Z + U * C + h * E, K[9] = A * Q + P * B + U * V + h * J, K[10] = A * X + P * O + U * H + h * I, K[11] = A * Y + P * G + U * L + h * k, A = $[12], P = $[13], U = $[14], h = $[15], K[12] = A * N + P * Z + U * C + h * E, K[13] = A * Q + P * B + U * V + h * J, K[14] = A * X + P * O + U * H + h * I, K[15] = A * Y + P * G + U * L + h * k, K;
};
var JK = function(K, W, $) {
  var N = $[0], Q = $[1], X = $[2], Y, Z, B, O, G, C, V, H, L, E, J, I;
  if (W === K)
    K[12] = W[0] * N + W[4] * Q + W[8] * X + W[12], K[13] = W[1] * N + W[5] * Q + W[9] * X + W[13], K[14] = W[2] * N + W[6] * Q + W[10] * X + W[14], K[15] = W[3] * N + W[7] * Q + W[11] * X + W[15];
  else
    Y = W[0], Z = W[1], B = W[2], O = W[3], G = W[4], C = W[5], V = W[6], H = W[7], L = W[8], E = W[9], J = W[10], I = W[11], K[0] = Y, K[1] = Z, K[2] = B, K[3] = O, K[4] = G, K[5] = C, K[6] = V, K[7] = H, K[8] = L, K[9] = E, K[10] = J, K[11] = I, K[12] = Y * N + G * Q + L * X + W[12], K[13] = Z * N + C * Q + E * X + W[13], K[14] = B * N + V * Q + J * X + W[14], K[15] = O * N + H * Q + I * X + W[15];
  return K;
};
var CK = function(K, W, $) {
  var N = $[0], Q = $[1], X = $[2];
  return K[0] = W[0] * N, K[1] = W[1] * N, K[2] = W[2] * N, K[3] = W[3] * N, K[4] = W[4] * Q, K[5] = W[5] * Q, K[6] = W[6] * Q, K[7] = W[7] * Q, K[8] = W[8] * X, K[9] = W[9] * X, K[10] = W[10] * X, K[11] = W[11] * X, K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15], K;
};
var HK = function(K, W, $, N) {
  var Q = N[0], X = N[1], Y = N[2], Z = Math.hypot(Q, X, Y), B, O, G, C, V, H, L, E, J, I, k, A, P, U, h, g, _, j, M, p, F, q, R, f;
  if (Z < D)
    return null;
  if (Z = 1 / Z, Q *= Z, X *= Z, Y *= Z, B = Math.sin($), O = Math.cos($), G = 1 - O, C = W[0], V = W[1], H = W[2], L = W[3], E = W[4], J = W[5], I = W[6], k = W[7], A = W[8], P = W[9], U = W[10], h = W[11], g = Q * Q * G + O, _ = X * Q * G + Y * B, j = Y * Q * G - X * B, M = Q * X * G - Y * B, p = X * X * G + O, F = Y * X * G + Q * B, q = Q * Y * G + X * B, R = X * Y * G - Q * B, f = Y * Y * G + O, K[0] = C * g + E * _ + A * j, K[1] = V * g + J * _ + P * j, K[2] = H * g + I * _ + U * j, K[3] = L * g + k * _ + h * j, K[4] = C * M + E * p + A * F, K[5] = V * M + J * p + P * F, K[6] = H * M + I * p + U * F, K[7] = L * M + k * p + h * F, K[8] = C * q + E * R + A * f, K[9] = V * q + J * R + P * f, K[10] = H * q + I * R + U * f, K[11] = L * q + k * R + h * f, W !== K)
    K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K;
};
var VK = function(K, W, $) {
  var N = Math.sin($), Q = Math.cos($), X = W[4], Y = W[5], Z = W[6], B = W[7], O = W[8], G = W[9], C = W[10], V = W[11];
  if (W !== K)
    K[0] = W[0], K[1] = W[1], K[2] = W[2], K[3] = W[3], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[4] = X * Q + O * N, K[5] = Y * Q + G * N, K[6] = Z * Q + C * N, K[7] = B * Q + V * N, K[8] = O * Q - X * N, K[9] = G * Q - Y * N, K[10] = C * Q - Z * N, K[11] = V * Q - B * N, K;
};
var EK = function(K, W, $) {
  var N = Math.sin($), Q = Math.cos($), X = W[0], Y = W[1], Z = W[2], B = W[3], O = W[8], G = W[9], C = W[10], V = W[11];
  if (W !== K)
    K[4] = W[4], K[5] = W[5], K[6] = W[6], K[7] = W[7], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[0] = X * Q - O * N, K[1] = Y * Q - G * N, K[2] = Z * Q - C * N, K[3] = B * Q - V * N, K[8] = X * N + O * Q, K[9] = Y * N + G * Q, K[10] = Z * N + C * Q, K[11] = B * N + V * Q, K;
};
var LK = function(K, W, $) {
  var N = Math.sin($), Q = Math.cos($), X = W[0], Y = W[1], Z = W[2], B = W[3], O = W[4], G = W[5], C = W[6], V = W[7];
  if (W !== K)
    K[8] = W[8], K[9] = W[9], K[10] = W[10], K[11] = W[11], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[0] = X * Q + O * N, K[1] = Y * Q + G * N, K[2] = Z * Q + C * N, K[3] = B * Q + V * N, K[4] = O * Q - X * N, K[5] = G * Q - Y * N, K[6] = C * Q - Z * N, K[7] = V * Q - B * N, K;
};
var IK = function(K, W) {
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = W[0], K[13] = W[1], K[14] = W[2], K[15] = 1, K;
};
var UK = function(K, W) {
  return K[0] = W[0], K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = W[1], K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = W[2], K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var hK = function(K, W, $) {
  var N = $[0], Q = $[1], X = $[2], Y = Math.hypot(N, Q, X), Z, B, O;
  if (Y < D)
    return null;
  return Y = 1 / Y, N *= Y, Q *= Y, X *= Y, Z = Math.sin(W), B = Math.cos(W), O = 1 - B, K[0] = N * N * O + B, K[1] = Q * N * O + X * Z, K[2] = X * N * O - Q * Z, K[3] = 0, K[4] = N * Q * O - X * Z, K[5] = Q * Q * O + B, K[6] = X * Q * O + N * Z, K[7] = 0, K[8] = N * X * O + Q * Z, K[9] = Q * X * O - N * Z, K[10] = X * X * O + B, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
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
  var N = W[0], Q = W[1], X = W[2], Y = W[3], Z = N + N, B = Q + Q, O = X + X, G = N * Z, C = N * B, V = N * O, H = Q * B, L = Q * O, E = X * O, J = Y * Z, I = Y * B, k = Y * O;
  return K[0] = 1 - (H + E), K[1] = C + k, K[2] = V - I, K[3] = 0, K[4] = C - k, K[5] = 1 - (G + E), K[6] = L + J, K[7] = 0, K[8] = V + I, K[9] = L - J, K[10] = 1 - (G + H), K[11] = 0, K[12] = $[0], K[13] = $[1], K[14] = $[2], K[15] = 1, K;
};
var kK = function(K, W) {
  var $ = new S(3), N = -W[0], Q = -W[1], X = -W[2], Y = W[3], Z = W[4], B = W[5], O = W[6], G = W[7], C = N * N + Q * Q + X * X + Y * Y;
  if (C > 0)
    $[0] = (Z * Y + G * N + B * X - O * Q) * 2 / C, $[1] = (B * Y + G * Q + O * N - Z * X) * 2 / C, $[2] = (O * Y + G * X + Z * Q - B * N) * 2 / C;
  else
    $[0] = (Z * Y + G * N + B * X - O * Q) * 2, $[1] = (B * Y + G * Q + O * N - Z * X) * 2, $[2] = (O * Y + G * X + Z * Q - B * N) * 2;
  return G0(K, W, $), K;
};
var RK = function(K, W) {
  return K[0] = W[12], K[1] = W[13], K[2] = W[14], K;
};
var J0 = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[4], Y = W[5], Z = W[6], B = W[8], O = W[9], G = W[10];
  return K[0] = Math.hypot($, N, Q), K[1] = Math.hypot(X, Y, Z), K[2] = Math.hypot(B, O, G), K;
};
var TK = function(K, W) {
  var $ = new S(3);
  J0($, W);
  var N = 1 / $[0], Q = 1 / $[1], X = 1 / $[2], Y = W[0] * N, Z = W[1] * Q, B = W[2] * X, O = W[4] * N, G = W[5] * Q, C = W[6] * X, V = W[8] * N, H = W[9] * Q, L = W[10] * X, E = Y + G + L, J = 0;
  if (E > 0)
    J = Math.sqrt(E + 1) * 2, K[3] = 0.25 * J, K[0] = (C - H) / J, K[1] = (V - B) / J, K[2] = (Z - O) / J;
  else if (Y > G && Y > L)
    J = Math.sqrt(1 + Y - G - L) * 2, K[3] = (C - H) / J, K[0] = 0.25 * J, K[1] = (Z + O) / J, K[2] = (V + B) / J;
  else if (G > L)
    J = Math.sqrt(1 + G - Y - L) * 2, K[3] = (V - B) / J, K[0] = (Z + O) / J, K[1] = 0.25 * J, K[2] = (C + H) / J;
  else
    J = Math.sqrt(1 + L - Y - G) * 2, K[3] = (Z - O) / J, K[0] = (V + B) / J, K[1] = (C + H) / J, K[2] = 0.25 * J;
  return K;
};
var SK = function(K, W, $, N) {
  var Q = W[0], X = W[1], Y = W[2], Z = W[3], B = Q + Q, O = X + X, G = Y + Y, C = Q * B, V = Q * O, H = Q * G, L = X * O, E = X * G, J = Y * G, I = Z * B, k = Z * O, A = Z * G, P = N[0], U = N[1], h = N[2];
  return K[0] = (1 - (L + J)) * P, K[1] = (V + A) * P, K[2] = (H - k) * P, K[3] = 0, K[4] = (V - A) * U, K[5] = (1 - (C + J)) * U, K[6] = (E + I) * U, K[7] = 0, K[8] = (H + k) * h, K[9] = (E - I) * h, K[10] = (1 - (C + L)) * h, K[11] = 0, K[12] = $[0], K[13] = $[1], K[14] = $[2], K[15] = 1, K;
};
var _K = function(K, W, $, N, Q) {
  var X = W[0], Y = W[1], Z = W[2], B = W[3], O = X + X, G = Y + Y, C = Z + Z, V = X * O, H = X * G, L = X * C, E = Y * G, J = Y * C, I = Z * C, k = B * O, A = B * G, P = B * C, U = N[0], h = N[1], g = N[2], _ = Q[0], j = Q[1], M = Q[2], p = (1 - (E + I)) * U, F = (H + P) * U, q = (L - A) * U, R = (H - P) * h, f = (1 - (V + I)) * h, l = (J + k) * h, c = (L + A) * g, X0 = (J - k) * g, Y0 = (1 - (V + E)) * g;
  return K[0] = p, K[1] = F, K[2] = q, K[3] = 0, K[4] = R, K[5] = f, K[6] = l, K[7] = 0, K[8] = c, K[9] = X0, K[10] = Y0, K[11] = 0, K[12] = $[0] + _ - (p * _ + R * j + c * M), K[13] = $[1] + j - (F * _ + f * j + X0 * M), K[14] = $[2] + M - (q * _ + l * j + Y0 * M), K[15] = 1, K;
};
var jK = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = $ + $, Z = N + N, B = Q + Q, O = $ * Y, G = N * Y, C = N * Z, V = Q * Y, H = Q * Z, L = Q * B, E = X * Y, J = X * Z, I = X * B;
  return K[0] = 1 - C - L, K[1] = G + I, K[2] = V - J, K[3] = 0, K[4] = G - I, K[5] = 1 - O - L, K[6] = H + E, K[7] = 0, K[8] = V + J, K[9] = H - E, K[10] = 1 - O - C, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var MK = function(K, W, $, N, Q, X, Y) {
  var Z = 1 / ($ - W), B = 1 / (Q - N), O = 1 / (X - Y);
  return K[0] = X * 2 * Z, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = X * 2 * B, K[6] = 0, K[7] = 0, K[8] = ($ + W) * Z, K[9] = (Q + N) * B, K[10] = (Y + X) * O, K[11] = -1, K[12] = 0, K[13] = 0, K[14] = Y * X * 2 * O, K[15] = 0, K;
};
var C0 = function(K, W, $, N, Q) {
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
  var Q = Math.tan(W.upDegrees * Math.PI / 180), X = Math.tan(W.downDegrees * Math.PI / 180), Y = Math.tan(W.leftDegrees * Math.PI / 180), Z = Math.tan(W.rightDegrees * Math.PI / 180), B = 2 / (Y + Z), O = 2 / (Q + X);
  return K[0] = B, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = O, K[6] = 0, K[7] = 0, K[8] = -((Y - Z) * B * 0.5), K[9] = (Q - X) * O * 0.5, K[10] = N / ($ - N), K[11] = -1, K[12] = 0, K[13] = 0, K[14] = N * $ / ($ - N), K[15] = 0, K;
};
var H0 = function(K, W, $, N, Q, X, Y) {
  var Z = 1 / (W - $), B = 1 / (N - Q), O = 1 / (X - Y);
  return K[0] = -2 * Z, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = -2 * B, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 2 * O, K[11] = 0, K[12] = (W + $) * Z, K[13] = (Q + N) * B, K[14] = (Y + X) * O, K[15] = 1, K;
};
var fK = function(K, W, $, N, Q, X, Y) {
  var Z = 1 / (W - $), B = 1 / (N - Q), O = 1 / (X - Y);
  return K[0] = -2 * Z, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = -2 * B, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = O, K[11] = 0, K[12] = (W + $) * Z, K[13] = (Q + N) * B, K[14] = X * O, K[15] = 1, K;
};
var wK = function(K, W, $, N) {
  var Q, X, Y, Z, B, O, G, C, V, H, L = W[0], E = W[1], J = W[2], I = N[0], k = N[1], A = N[2], P = $[0], U = $[1], h = $[2];
  if (Math.abs(L - P) < D && Math.abs(E - U) < D && Math.abs(J - h) < D)
    return B0(K);
  if (G = L - P, C = E - U, V = J - h, H = 1 / Math.hypot(G, C, V), G *= H, C *= H, V *= H, Q = k * V - A * C, X = A * G - I * V, Y = I * C - k * G, H = Math.hypot(Q, X, Y), !H)
    Q = 0, X = 0, Y = 0;
  else
    H = 1 / H, Q *= H, X *= H, Y *= H;
  if (Z = C * Y - V * X, B = V * Q - G * Y, O = G * X - C * Q, H = Math.hypot(Z, B, O), !H)
    Z = 0, B = 0, O = 0;
  else
    H = 1 / H, Z *= H, B *= H, O *= H;
  return K[0] = Q, K[1] = Z, K[2] = G, K[3] = 0, K[4] = X, K[5] = B, K[6] = C, K[7] = 0, K[8] = Y, K[9] = O, K[10] = V, K[11] = 0, K[12] = -(Q * L + X * E + Y * J), K[13] = -(Z * L + B * E + O * J), K[14] = -(G * L + C * E + V * J), K[15] = 1, K;
};
var nK = function(K, W, $, N) {
  var Q = W[0], X = W[1], Y = W[2], Z = N[0], B = N[1], O = N[2], G = Q - $[0], C = X - $[1], V = Y - $[2], H = G * G + C * C + V * V;
  if (H > 0)
    H = 1 / Math.sqrt(H), G *= H, C *= H, V *= H;
  var L = B * V - O * C, E = O * G - Z * V, J = Z * C - B * G;
  if (H = L * L + E * E + J * J, H > 0)
    H = 1 / Math.sqrt(H), L *= H, E *= H, J *= H;
  return K[0] = L, K[1] = E, K[2] = J, K[3] = 0, K[4] = C * J - V * E, K[5] = V * L - G * J, K[6] = G * E - C * L, K[7] = 0, K[8] = G, K[9] = C, K[10] = V, K[11] = 0, K[12] = Q, K[13] = X, K[14] = Y, K[15] = 1, K;
};
var dK = function(K) {
  return "mat4(" + K[0] + ", " + K[1] + ", " + K[2] + ", " + K[3] + ", " + K[4] + ", " + K[5] + ", " + K[6] + ", " + K[7] + ", " + K[8] + ", " + K[9] + ", " + K[10] + ", " + K[11] + ", " + K[12] + ", " + K[13] + ", " + K[14] + ", " + K[15] + ")";
};
var vK = function(K) {
  return Math.hypot(K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8], K[9], K[10], K[11], K[12], K[13], K[14], K[15]);
};
var iK = function(K, W, $) {
  return K[0] = W[0] + $[0], K[1] = W[1] + $[1], K[2] = W[2] + $[2], K[3] = W[3] + $[3], K[4] = W[4] + $[4], K[5] = W[5] + $[5], K[6] = W[6] + $[6], K[7] = W[7] + $[7], K[8] = W[8] + $[8], K[9] = W[9] + $[9], K[10] = W[10] + $[10], K[11] = W[11] + $[11], K[12] = W[12] + $[12], K[13] = W[13] + $[13], K[14] = W[14] + $[14], K[15] = W[15] + $[15], K;
};
var V0 = function(K, W, $) {
  return K[0] = W[0] - $[0], K[1] = W[1] - $[1], K[2] = W[2] - $[2], K[3] = W[3] - $[3], K[4] = W[4] - $[4], K[5] = W[5] - $[5], K[6] = W[6] - $[6], K[7] = W[7] - $[7], K[8] = W[8] - $[8], K[9] = W[9] - $[9], K[10] = W[10] - $[10], K[11] = W[11] - $[11], K[12] = W[12] - $[12], K[13] = W[13] - $[13], K[14] = W[14] - $[14], K[15] = W[15] - $[15], K;
};
var lK = function(K, W, $) {
  return K[0] = W[0] * $, K[1] = W[1] * $, K[2] = W[2] * $, K[3] = W[3] * $, K[4] = W[4] * $, K[5] = W[5] * $, K[6] = W[6] * $, K[7] = W[7] * $, K[8] = W[8] * $, K[9] = W[9] * $, K[10] = W[10] * $, K[11] = W[11] * $, K[12] = W[12] * $, K[13] = W[13] * $, K[14] = W[14] * $, K[15] = W[15] * $, K;
};
var cK = function(K, W, $, N) {
  return K[0] = W[0] + $[0] * N, K[1] = W[1] + $[1] * N, K[2] = W[2] + $[2] * N, K[3] = W[3] + $[3] * N, K[4] = W[4] + $[4] * N, K[5] = W[5] + $[5] * N, K[6] = W[6] + $[6] * N, K[7] = W[7] + $[7] * N, K[8] = W[8] + $[8] * N, K[9] = W[9] + $[9] * N, K[10] = W[10] + $[10] * N, K[11] = W[11] + $[11] * N, K[12] = W[12] + $[12] * N, K[13] = W[13] + $[13] * N, K[14] = W[14] + $[14] * N, K[15] = W[15] + $[15] * N, K;
};
var yK = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2] && K[3] === W[3] && K[4] === W[4] && K[5] === W[5] && K[6] === W[6] && K[7] === W[7] && K[8] === W[8] && K[9] === W[9] && K[10] === W[10] && K[11] === W[11] && K[12] === W[12] && K[13] === W[13] && K[14] === W[14] && K[15] === W[15];
};
var rK = function(K, W) {
  var $ = K[0], N = K[1], Q = K[2], X = K[3], Y = K[4], Z = K[5], B = K[6], O = K[7], G = K[8], C = K[9], V = K[10], H = K[11], L = K[12], E = K[13], J = K[14], I = K[15], k = W[0], A = W[1], P = W[2], U = W[3], h = W[4], g = W[5], _ = W[6], j = W[7], M = W[8], p = W[9], F = W[10], q = W[11], R = W[12], f = W[13], l = W[14], c = W[15];
  return Math.abs($ - k) <= D * Math.max(1, Math.abs($), Math.abs(k)) && Math.abs(N - A) <= D * Math.max(1, Math.abs(N), Math.abs(A)) && Math.abs(Q - P) <= D * Math.max(1, Math.abs(Q), Math.abs(P)) && Math.abs(X - U) <= D * Math.max(1, Math.abs(X), Math.abs(U)) && Math.abs(Y - h) <= D * Math.max(1, Math.abs(Y), Math.abs(h)) && Math.abs(Z - g) <= D * Math.max(1, Math.abs(Z), Math.abs(g)) && Math.abs(B - _) <= D * Math.max(1, Math.abs(B), Math.abs(_)) && Math.abs(O - j) <= D * Math.max(1, Math.abs(O), Math.abs(j)) && Math.abs(G - M) <= D * Math.max(1, Math.abs(G), Math.abs(M)) && Math.abs(C - p) <= D * Math.max(1, Math.abs(C), Math.abs(p)) && Math.abs(V - F) <= D * Math.max(1, Math.abs(V), Math.abs(F)) && Math.abs(H - q) <= D * Math.max(1, Math.abs(H), Math.abs(q)) && Math.abs(L - R) <= D * Math.max(1, Math.abs(L), Math.abs(R)) && Math.abs(E - f) <= D * Math.max(1, Math.abs(E), Math.abs(f)) && Math.abs(J - l) <= D * Math.max(1, Math.abs(J), Math.abs(l)) && Math.abs(I - c) <= D * Math.max(1, Math.abs(I), Math.abs(c));
};
var x = function() {
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
var e = function(K, W, $) {
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
var I0 = function(K, W, $) {
  return K[0] = W[0] * $[0], K[1] = W[1] * $[1], K[2] = W[2] * $[2], K;
};
var U0 = function(K, W, $) {
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
var h0 = function(K, W) {
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
var b = function(K, W) {
  return K[0] * W[0] + K[1] * W[1] + K[2] * W[2];
};
var r = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2], Y = $[0], Z = $[1], B = $[2];
  return K[0] = Q * B - X * Z, K[1] = X * Y - N * B, K[2] = N * Z - Q * Y, K;
};
var XW = function(K, W, $, N) {
  var Q = W[0], X = W[1], Y = W[2];
  return K[0] = Q + N * ($[0] - Q), K[1] = X + N * ($[1] - X), K[2] = Y + N * ($[2] - Y), K;
};
var YW = function(K, W, $, N, Q, X) {
  var Y = X * X, Z = Y * (2 * X - 3) + 1, B = Y * (X - 2) + X, O = Y * (X - 1), G = Y * (3 - 2 * X);
  return K[0] = W[0] * Z + $[0] * B + N[0] * O + Q[0] * G, K[1] = W[1] * Z + $[1] * B + N[1] * O + Q[1] * G, K[2] = W[2] * Z + $[2] * B + N[2] * O + Q[2] * G, K;
};
var ZW = function(K, W, $, N, Q, X) {
  var Y = 1 - X, Z = Y * Y, B = X * X, O = Z * Y, G = 3 * X * Z, C = 3 * B * Y, V = B * X;
  return K[0] = W[0] * O + $[0] * G + N[0] * C + Q[0] * V, K[1] = W[1] * O + $[1] * G + N[1] * C + Q[1] * V, K[2] = W[2] * O + $[2] * G + N[2] * C + Q[2] * V, K;
};
var BW = function(K, W) {
  W = W || 1;
  var $ = d() * 2 * Math.PI, N = d() * 2 - 1, Q = Math.sqrt(1 - N * N) * W;
  return K[0] = Math.cos($) * Q, K[1] = Math.sin($) * Q, K[2] = N * W, K;
};
var OW = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2], Y = $[3] * N + $[7] * Q + $[11] * X + $[15];
  return Y = Y || 1, K[0] = ($[0] * N + $[4] * Q + $[8] * X + $[12]) / Y, K[1] = ($[1] * N + $[5] * Q + $[9] * X + $[13]) / Y, K[2] = ($[2] * N + $[6] * Q + $[10] * X + $[14]) / Y, K;
};
var GW = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2];
  return K[0] = N * $[0] + Q * $[3] + X * $[6], K[1] = N * $[1] + Q * $[4] + X * $[7], K[2] = N * $[2] + Q * $[5] + X * $[8], K;
};
var JW = function(K, W, $) {
  var N = $[0], Q = $[1], X = $[2], Y = $[3], Z = W[0], B = W[1], O = W[2], G = Q * O - X * B, C = X * Z - N * O, V = N * B - Q * Z, H = Q * V - X * C, L = X * G - N * V, E = N * C - Q * G, J = Y * 2;
  return G *= J, C *= J, V *= J, H *= 2, L *= 2, E *= 2, K[0] = Z + G + H, K[1] = B + C + L, K[2] = O + V + E, K;
};
var CW = function(K, W, $, N) {
  var Q = [], X = [];
  return Q[0] = W[0] - $[0], Q[1] = W[1] - $[1], Q[2] = W[2] - $[2], X[0] = Q[0], X[1] = Q[1] * Math.cos(N) - Q[2] * Math.sin(N), X[2] = Q[1] * Math.sin(N) + Q[2] * Math.cos(N), K[0] = X[0] + $[0], K[1] = X[1] + $[1], K[2] = X[2] + $[2], K;
};
var HW = function(K, W, $, N) {
  var Q = [], X = [];
  return Q[0] = W[0] - $[0], Q[1] = W[1] - $[1], Q[2] = W[2] - $[2], X[0] = Q[2] * Math.sin(N) + Q[0] * Math.cos(N), X[1] = Q[1], X[2] = Q[2] * Math.cos(N) - Q[0] * Math.sin(N), K[0] = X[0] + $[0], K[1] = X[1] + $[1], K[2] = X[2] + $[2], K;
};
var VW = function(K, W, $, N) {
  var Q = [], X = [];
  return Q[0] = W[0] - $[0], Q[1] = W[1] - $[1], Q[2] = W[2] - $[2], X[0] = Q[0] * Math.cos(N) - Q[1] * Math.sin(N), X[1] = Q[0] * Math.sin(N) + Q[1] * Math.cos(N), X[2] = Q[2], K[0] = X[0] + $[0], K[1] = X[1] + $[1], K[2] = X[2] + $[2], K;
};
var EW = function(K, W) {
  var $ = K[0], N = K[1], Q = K[2], X = W[0], Y = W[1], Z = W[2], B = Math.sqrt($ * $ + N * N + Q * Q), O = Math.sqrt(X * X + Y * Y + Z * Z), G = B * O, C = G && b(K, W) / G;
  return Math.acos(Math.min(Math.max(C, -1), 1));
};
var LW = function(K) {
  return K[0] = 0, K[1] = 0, K[2] = 0, K;
};
var IW = function(K) {
  return "vec3(" + K[0] + ", " + K[1] + ", " + K[2] + ")";
};
var UW = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2];
};
var hW = function(K, W) {
  var $ = K[0], N = K[1], Q = K[2], X = W[0], Y = W[1], Z = W[2];
  return Math.abs($ - X) <= D * Math.max(1, Math.abs($), Math.abs(X)) && Math.abs(N - Y) <= D * Math.max(1, Math.abs(N), Math.abs(Y)) && Math.abs(Q - Z) <= D * Math.max(1, Math.abs(Q), Math.abs(Z));
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
var R0 = function(K, W) {
  return K[0] = W[0], K[1] = W[1], K[2] = W[2], K[3] = W[3], K;
};
var T0 = function(K, W, $, N, Q) {
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
  var Q = W[0], X = W[1], Y = W[2], Z = W[3];
  return K[0] = Q + N * ($[0] - Q), K[1] = X + N * ($[1] - X), K[2] = Y + N * ($[2] - Y), K[3] = Z + N * ($[3] - Z), K;
};
var q0 = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2] && K[3] === W[3];
};
var f0 = function(K, W) {
  var $ = K[0], N = K[1], Q = K[2], X = K[3], Y = W[0], Z = W[1], B = W[2], O = W[3];
  return Math.abs($ - Y) <= D * Math.max(1, Math.abs($), Math.abs(Y)) && Math.abs(N - Z) <= D * Math.max(1, Math.abs(N), Math.abs(Z)) && Math.abs(Q - B) <= D * Math.max(1, Math.abs(Q), Math.abs(B)) && Math.abs(X - O) <= D * Math.max(1, Math.abs(X), Math.abs(O));
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
var n0 = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2], Y = W[3], Z = $[0], B = $[1], O = $[2], G = $[3];
  return K[0] = N * G + Y * Z + Q * O - X * B, K[1] = Q * G + Y * B + X * Z - N * O, K[2] = X * G + Y * O + N * B - Q * Z, K[3] = Y * G - N * Z - Q * B - X * O, K;
};
var FW = function(K, W, $) {
  $ *= 0.5;
  var N = W[0], Q = W[1], X = W[2], Y = W[3], Z = Math.sin($), B = Math.cos($);
  return K[0] = N * B + Y * Z, K[1] = Q * B + X * Z, K[2] = X * B - Q * Z, K[3] = Y * B - N * Z, K;
};
var qW = function(K, W, $) {
  $ *= 0.5;
  var N = W[0], Q = W[1], X = W[2], Y = W[3], Z = Math.sin($), B = Math.cos($);
  return K[0] = N * B - X * Z, K[1] = Q * B + Y * Z, K[2] = X * B + N * Z, K[3] = Y * B - Q * Z, K;
};
var fW = function(K, W, $) {
  $ *= 0.5;
  var N = W[0], Q = W[1], X = W[2], Y = W[3], Z = Math.sin($), B = Math.cos($);
  return K[0] = N * B + Q * Z, K[1] = Q * B - N * Z, K[2] = X * B + Y * Z, K[3] = Y * B - X * Z, K;
};
var wW = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2];
  return K[0] = $, K[1] = N, K[2] = Q, K[3] = Math.sqrt(Math.abs(1 - $ * $ - N * N - Q * Q)), K;
};
var d0 = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = Math.sqrt($ * $ + N * N + Q * Q), Z = Math.exp(X), B = Y > 0 ? Z * Math.sin(Y) / Y : 0;
  return K[0] = $ * B, K[1] = N * B, K[2] = Q * B, K[3] = Z * Math.cos(Y), K;
};
var v0 = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = Math.sqrt($ * $ + N * N + Q * Q), Z = Y > 0 ? Math.atan2(Y, X) / Y : 0;
  return K[0] = $ * Z, K[1] = N * Z, K[2] = Q * Z, K[3] = 0.5 * Math.log($ * $ + N * N + Q * Q + X * X), K;
};
var nW = function(K, W, $) {
  return v0(K, W), l0(K, K, $), d0(K, K), K;
};
var u = function(K, W, $, N) {
  var Q = W[0], X = W[1], Y = W[2], Z = W[3], B = $[0], O = $[1], G = $[2], C = $[3], V, H, L, E, J;
  if (H = Q * B + X * O + Y * G + Z * C, H < 0)
    H = -H, B = -B, O = -O, G = -G, C = -C;
  if (1 - H > D)
    V = Math.acos(H), L = Math.sin(V), E = Math.sin((1 - N) * V) / L, J = Math.sin(N * V) / L;
  else
    E = 1 - N, J = N;
  return K[0] = E * Q + J * B, K[1] = E * X + J * O, K[2] = E * Y + J * G, K[3] = E * Z + J * C, K;
};
var dW = function(K) {
  var W = d(), $ = d(), N = d(), Q = Math.sqrt(1 - W), X = Math.sqrt(W);
  return K[0] = Q * Math.sin(2 * Math.PI * $), K[1] = Q * Math.cos(2 * Math.PI * $), K[2] = X * Math.sin(2 * Math.PI * N), K[3] = X * Math.cos(2 * Math.PI * N), K;
};
var vW = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = $ * $ + N * N + Q * Q + X * X, Z = Y ? 1 / Y : 0;
  return K[0] = -$ * Z, K[1] = -N * Z, K[2] = -Q * Z, K[3] = X * Z, K;
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
  var X = Math.sin(W), Y = Math.cos(W), Z = Math.sin($), B = Math.cos($), O = Math.sin(N), G = Math.cos(N);
  return K[0] = X * B * G - Y * Z * O, K[1] = Y * Z * G + X * B * O, K[2] = Y * B * O - X * Z * G, K[3] = Y * B * G + X * Z * O, K;
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
var T = {};
t(T, { transpose: () => {
  {
    return ZK;
  }
}, translate: () => {
  {
    return JK;
  }
}, targetTo: () => {
  {
    return nK;
  }
}, subtract: () => {
  {
    return V0;
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
    return CK;
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
    return VK;
  }
}, rotate: () => {
  {
    return HK;
  }
}, perspectiveZO: () => {
  {
    return pK;
  }
}, perspectiveNO: () => {
  {
    return C0;
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
    return H0;
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
    return O0;
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
    return BK;
  }
}, identity: () => {
  {
    return B0;
  }
}, getTranslation: () => {
  {
    return RK;
  }
}, getScaling: () => {
  {
    return J0;
  }
}, getRotation: () => {
  {
    return TK;
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
    return IK;
  }
}, fromScaling: () => {
  {
    return UK;
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
    return hK;
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
    return vK;
  }
}, exactEquals: () => {
  {
    return yK;
  }
}, equals: () => {
  {
    return rK;
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
    return OK;
  }
}, add: () => {
  {
    return iK;
  }
} });
var gK = C0;
var qK = H0;
var zK = O0;
var sK = V0;
var s = {};
t(s, { str: () => {
  {
    return cW;
  }
}, squaredLength: () => {
  {
    return r0;
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
    return u;
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
    return nW;
  }
}, normalize: () => {
  {
    return $0;
  }
}, multiply: () => {
  {
    return n0;
  }
}, mul: () => {
  {
    return xW;
  }
}, ln: () => {
  {
    return v0;
  }
}, lerp: () => {
  {
    return eW;
  }
}, length: () => {
  {
    return y0;
  }
}, len: () => {
  {
    return bW;
  }
}, invert: () => {
  {
    return vW;
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
    return rW;
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
    return yW;
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
var z = {};
t(z, { zero: () => {
  {
    return LW;
  }
}, transformQuat: () => {
  {
    return JW;
  }
}, transformMat4: () => {
  {
    return OW;
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
    return IW;
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
    return TW;
  }
}, sqrDist: () => {
  {
    return RW;
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
    return VW;
  }
}, rotateY: () => {
  {
    return HW;
  }
}, rotateX: () => {
  {
    return CW;
  }
}, random: () => {
  {
    return BW;
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
    return I0;
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
    return e;
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
    return UW;
  }
}, equals: () => {
  {
    return hW;
  }
}, dot: () => {
  {
    return b;
  }
}, divide: () => {
  {
    return U0;
  }
}, div: () => {
  {
    return DW;
  }
}, distance: () => {
  {
    return h0;
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
    return x;
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
    return ZW;
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
var AW = I0;
var DW = U0;
var kW = h0;
var RW = P0;
var K0 = E0;
var TW = A0;
var SW = function() {
  var K = x();
  return function(W, $, N, Q, X, Y) {
    var Z, B;
    if (!$)
      $ = 3;
    if (!N)
      N = 0;
    if (Q)
      B = Math.min(Q * $ + N, W.length);
    else
      B = W.length;
    for (Z = N;Z < B; Z += $)
      K[0] = W[Z], K[1] = W[Z + 1], K[2] = W[Z + 2], X(K, K, Y), W[Z] = K[0], W[Z + 1] = K[1], W[Z + 2] = K[2];
    return W;
  };
}();
var J$ = function() {
  var K = _W();
  return function(W, $, N, Q, X, Y) {
    var Z, B;
    if (!$)
      $ = 4;
    if (!N)
      N = 0;
    if (Q)
      B = Math.min(Q * $ + N, W.length);
    else
      B = W.length;
    for (Z = N;Z < B; Z += $)
      K[0] = W[Z], K[1] = W[Z + 1], K[2] = W[Z + 2], K[3] = W[Z + 3], X(K, K, Y), W[Z] = K[0], W[Z + 1] = K[1], W[Z + 2] = K[2], W[Z + 3] = K[3];
    return W;
  };
}();
var yW = D0;
var rW = k0;
var zW = R0;
var sW = T0;
var mW = S0;
var xW = n0;
var l0 = _0;
var c0 = p0;
var eW = F0;
var y0 = j0;
var bW = y0;
var r0 = M0;
var uW = r0;
var $0 = g0;
var oW = q0;
var tW = f0;
var aW = function() {
  var K = x(), W = e(1, 0, 0), $ = e(0, 1, 0);
  return function(N, Q, X) {
    var Y = b(Q, X);
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
  return function($, N, Q, X, Y, Z) {
    return u(K, N, Y, Z), u(W, Q, X, Z), u($, K, W, 2 * Z * (1 - Z)), $;
  };
}();
var W$ = function() {
  var K = Z0();
  return function(W, $, N, Q) {
    return K[0] = N[0], K[3] = N[1], K[6] = N[2], K[1] = Q[0], K[4] = Q[1], K[7] = Q[2], K[2] = -$[0], K[5] = -$[1], K[8] = -$[2], $0(W, i0(W, K));
  };
}();
var $$ = Math.PI / 90;
var N0 = [0, 0, 0];
var z0 = T.create();
var s0 = T.create();
var o = s.create();

class m {
  #K = Float32Array.from(T.create());
  static HIDDEN = m.create().scale(0, 0, 0);
  static IDENTITY = m.create();
  constructor() {
    this.identity();
  }
  static create() {
    return new m;
  }
  copy(K) {
    return T.copy(this.#K, K.getMatrix()), this;
  }
  identity() {
    return T.identity(this.#K), this;
  }
  invert(K) {
    return T.invert(this.#K, K?.getMatrix() ?? this.getMatrix()), this;
  }
  multiply(K) {
    return T.multiply(this.#K, this.#K, K.getMatrix()), this;
  }
  multiply2(K, W) {
    return T.multiply(this.#K, K.getMatrix(), W.getMatrix()), this;
  }
  multiply3(K, W, $) {
    return this.multiply2(K, W), this.multiply($), this;
  }
  translate(K, W, $) {
    const N = N0;
    return N[0] = K, N[1] = W, N[2] = $, this.move(N);
  }
  move(K) {
    return T.translate(this.#K, this.#K, K), this;
  }
  rotateX(K) {
    return T.rotateX(this.#K, this.#K, K), this;
  }
  rotateY(K) {
    return T.rotateY(this.#K, this.#K, K), this;
  }
  rotateZ(K) {
    return T.rotateZ(this.#K, this.#K, K), this;
  }
  setXRotation(K) {
    return T.fromXRotation(this.getMatrix(), K), this;
  }
  setYRotation(K) {
    return T.fromYRotation(this.getMatrix(), K), this;
  }
  scale(K, W, $) {
    return T.scale(this.#K, this.#K, [K, W ?? K, $ ?? K]), this;
  }
  perspective(K, W, $, N) {
    return T.perspective(this.#K, K * $$, W, $, N), this;
  }
  ortho(K, W, $, N, Q, X) {
    return T.ortho(this.#K, K, W, $, N, Q, X), this;
  }
  combine(K, W, $ = 0.5) {
    return T.multiplyScalar(z0, K.getMatrix(), 1 - $), T.multiplyScalar(s0, W.getMatrix(), $), T.add(this.#K, z0, s0), this;
  }
  static getMoveVector(K, W, $, N) {
    const Q = N0;
    if (Q[0] = K, Q[1] = W, Q[2] = $, N)
      T.getRotation(o, N.getMatrix()), s.invert(o, o), z.transformQuat(Q, Q, o);
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
var w = m;

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

class v {
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

class i {
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
  static toVector(K, W, $, N) {
    return N[0] = K, N[1] = W, N[2] = $, N;
  }
  static transformToPosition(K, W) {
    const $ = K.getMatrix();
    W[0] = $[12], W[1] = $[13], W[2] = $[14];
  }
}
var n;
(function(N) {
  N[N["AT_POSITION"] = 0] = "AT_POSITION";
  N[N["MOVED"] = 1] = "MOVED";
  N[N["BLOCKED"] = 2] = "BLOCKED";
})(n || (n = {}));

class o0 {
  #K = w.create().setPosition(0, 0, 0);
  #W = new Set;
  #$ = [0, 0, 0];
  position = [0, 0, 0];
  moveBlocker;
  constructor({ blocker: K }, W) {
    if (this.moveBlocker = K, W)
      this.onChange(W);
  }
  onChange(K) {
    return this.#W.add(K), this;
  }
  removeChangeListener(K) {
    this.#W.delete(K);
  }
  changedPosition(K, W, $) {
    i.transformToPosition(this.#K, this.position);
    for (let N of this.#W)
      N(K, W, $);
  }
  moveBy(K, W, $, N) {
    const Q = w.getMoveVector(K, W, $, N), X = this.moveBlocker?.isBlocked(i.toVector(this.position[0] + Q[0], this.position[1] + Q[1], this.position[2] + Q[2], this.#$), this.position);
    if (!X)
      if (Q[0] || Q[1] || Q[2])
        this.#K.move(Q), this.changedPosition(K, W, $);
      else
        return n.AT_POSITION;
    return X ? n.BLOCKED : n.MOVED;
  }
  moveTo(K, W, $) {
    if (this.position[0] === K && this.position[1] === W && this.position[2] === $)
      return n.AT_POSITION;
    const N = this.moveBlocker?.isBlocked(i.toVector(K, W, $, this.#$), this.position);
    if (!N) {
      const [Q, X, Y] = this.#K.getPosition();
      if (Q !== K || X !== W || Y !== $) {
        const Z = K - Q, B = W - X, O = $ - Y;
        this.#K.setPosition(K, W, $), this.changedPosition(Z, B, O);
      }
    }
    return N ? n.BLOCKED : n.MOVED;
  }
  movedTo(K, W, $) {
    return this.moveTo(K, W, $), this;
  }
  gotoPos(K, W, $, N = 0.1) {
    const Q = this.position, X = K - Q[0], Y = W - Q[1], Z = $ - Q[2], B = Math.sqrt(X * X + Y * Y + Z * Z);
    if (B > 0.01) {
      const O = Math.min(B, N);
      return this.moveBy(X / B * O, Y / B * O, Z / B * O);
    } else
      return this.moveTo(K, W, $);
  }
  getMatrix() {
    return this.#K.getMatrix();
  }
}
var Y$ = 1;
var Z$ = 1;

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
    this.perspective = new v(Y$, K), this.zoom = new v(Z$, (W) => {
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
