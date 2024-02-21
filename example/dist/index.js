// /Users/vincent/dok-matrix/example/node_modules/dok-matrix/dist/index.js
var O0 = function() {
  var K = new S(9);
  if (S != Float32Array)
    K[1] = 0, K[2] = 0, K[3] = 0, K[5] = 0, K[6] = 0, K[7] = 0;
  return K[0] = 1, K[4] = 1, K[8] = 1, K;
};
var NK = function() {
  var K = new S(16);
  if (S != Float32Array)
    K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0;
  return K[0] = 1, K[5] = 1, K[10] = 1, K[15] = 1, K;
};
var QK = function(K) {
  var W = new S(16);
  return W[0] = K[0], W[1] = K[1], W[2] = K[2], W[3] = K[3], W[4] = K[4], W[5] = K[5], W[6] = K[6], W[7] = K[7], W[8] = K[8], W[9] = K[9], W[10] = K[10], W[11] = K[11], W[12] = K[12], W[13] = K[13], W[14] = K[14], W[15] = K[15], W;
};
var XK = function(K, W) {
  return K[0] = W[0], K[1] = W[1], K[2] = W[2], K[3] = W[3], K[4] = W[4], K[5] = W[5], K[6] = W[6], K[7] = W[7], K[8] = W[8], K[9] = W[9], K[10] = W[10], K[11] = W[11], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15], K;
};
var YK = function(K, W, $, N, Q, X, Y, B, O, Z, G, C, V, H, E, L) {
  var J = new S(16);
  return J[0] = K, J[1] = W, J[2] = $, J[3] = N, J[4] = Q, J[5] = X, J[6] = Y, J[7] = B, J[8] = O, J[9] = Z, J[10] = G, J[11] = C, J[12] = V, J[13] = H, J[14] = E, J[15] = L, J;
};
var BK = function(K, W, $, N, Q, X, Y, B, O, Z, G, C, V, H, E, L, J) {
  return K[0] = W, K[1] = $, K[2] = N, K[3] = Q, K[4] = X, K[5] = Y, K[6] = B, K[7] = O, K[8] = Z, K[9] = G, K[10] = C, K[11] = V, K[12] = H, K[13] = E, K[14] = L, K[15] = J, K;
};
var Z0 = function(K) {
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var OK = function(K, W) {
  if (K === W) {
    var $ = W[1], N = W[2], Q = W[3], X = W[6], Y = W[7], B = W[11];
    K[1] = W[4], K[2] = W[8], K[3] = W[12], K[4] = $, K[6] = W[9], K[7] = W[13], K[8] = N, K[9] = X, K[11] = W[14], K[12] = Q, K[13] = Y, K[14] = B;
  } else
    K[0] = W[0], K[1] = W[4], K[2] = W[8], K[3] = W[12], K[4] = W[1], K[5] = W[5], K[6] = W[9], K[7] = W[13], K[8] = W[2], K[9] = W[6], K[10] = W[10], K[11] = W[14], K[12] = W[3], K[13] = W[7], K[14] = W[11], K[15] = W[15];
  return K;
};
var ZK = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = W[4], B = W[5], O = W[6], Z = W[7], G = W[8], C = W[9], V = W[10], H = W[11], E = W[12], L = W[13], J = W[14], h = W[15], k = $ * B - N * Y, A = $ * O - Q * Y, P = $ * Z - X * Y, I = N * O - Q * B, U = N * Z - X * B, g = Q * Z - X * O, _ = G * L - C * E, j = G * J - V * E, M = G * h - H * E, p = C * J - V * L, F = C * h - H * L, q = V * h - H * J, R = k * q - A * F + P * p + I * M - U * j + g * _;
  if (!R)
    return null;
  return R = 1 / R, K[0] = (B * q - O * F + Z * p) * R, K[1] = (Q * F - N * q - X * p) * R, K[2] = (L * g - J * U + h * I) * R, K[3] = (V * U - C * g - H * I) * R, K[4] = (O * M - Y * q - Z * j) * R, K[5] = ($ * q - Q * M + X * j) * R, K[6] = (J * P - E * g - h * A) * R, K[7] = (G * g - V * P + H * A) * R, K[8] = (Y * F - B * M + Z * _) * R, K[9] = (N * M - $ * F - X * _) * R, K[10] = (E * U - L * P + h * k) * R, K[11] = (C * P - G * U - H * k) * R, K[12] = (B * j - Y * p - O * _) * R, K[13] = ($ * p - N * j + Q * _) * R, K[14] = (L * A - E * I - J * k) * R, K[15] = (G * I - C * A + V * k) * R, K;
};
var GK = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = W[4], B = W[5], O = W[6], Z = W[7], G = W[8], C = W[9], V = W[10], H = W[11], E = W[12], L = W[13], J = W[14], h = W[15];
  return K[0] = B * (V * h - H * J) - C * (O * h - Z * J) + L * (O * H - Z * V), K[1] = -(N * (V * h - H * J) - C * (Q * h - X * J) + L * (Q * H - X * V)), K[2] = N * (O * h - Z * J) - B * (Q * h - X * J) + L * (Q * Z - X * O), K[3] = -(N * (O * H - Z * V) - B * (Q * H - X * V) + C * (Q * Z - X * O)), K[4] = -(Y * (V * h - H * J) - G * (O * h - Z * J) + E * (O * H - Z * V)), K[5] = $ * (V * h - H * J) - G * (Q * h - X * J) + E * (Q * H - X * V), K[6] = -($ * (O * h - Z * J) - Y * (Q * h - X * J) + E * (Q * Z - X * O)), K[7] = $ * (O * H - Z * V) - Y * (Q * H - X * V) + G * (Q * Z - X * O), K[8] = Y * (C * h - H * L) - G * (B * h - Z * L) + E * (B * H - Z * C), K[9] = -($ * (C * h - H * L) - G * (N * h - X * L) + E * (N * H - X * C)), K[10] = $ * (B * h - Z * L) - Y * (N * h - X * L) + E * (N * Z - X * B), K[11] = -($ * (B * H - Z * C) - Y * (N * H - X * C) + G * (N * Z - X * B)), K[12] = -(Y * (C * J - V * L) - G * (B * J - O * L) + E * (B * V - O * C)), K[13] = $ * (C * J - V * L) - G * (N * J - Q * L) + E * (N * V - Q * C), K[14] = -($ * (B * J - O * L) - Y * (N * J - Q * L) + E * (N * O - Q * B)), K[15] = $ * (B * V - O * C) - Y * (N * V - Q * C) + G * (N * O - Q * B), K;
};
var JK = function(K) {
  var W = K[0], $ = K[1], N = K[2], Q = K[3], X = K[4], Y = K[5], B = K[6], O = K[7], Z = K[8], G = K[9], C = K[10], V = K[11], H = K[12], E = K[13], L = K[14], J = K[15], h = W * Y - $ * X, k = W * B - N * X, A = W * O - Q * X, P = $ * B - N * Y, I = $ * O - Q * Y, U = N * O - Q * B, g = Z * E - G * H, _ = Z * L - C * H, j = Z * J - V * H, M = G * L - C * E, p = G * J - V * E, F = C * J - V * L;
  return h * F - k * p + A * M + P * j - I * _ + U * g;
};
var G0 = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2], Y = W[3], B = W[4], O = W[5], Z = W[6], G = W[7], C = W[8], V = W[9], H = W[10], E = W[11], L = W[12], J = W[13], h = W[14], k = W[15], A = $[0], P = $[1], I = $[2], U = $[3];
  return K[0] = A * N + P * B + I * C + U * L, K[1] = A * Q + P * O + I * V + U * J, K[2] = A * X + P * Z + I * H + U * h, K[3] = A * Y + P * G + I * E + U * k, A = $[4], P = $[5], I = $[6], U = $[7], K[4] = A * N + P * B + I * C + U * L, K[5] = A * Q + P * O + I * V + U * J, K[6] = A * X + P * Z + I * H + U * h, K[7] = A * Y + P * G + I * E + U * k, A = $[8], P = $[9], I = $[10], U = $[11], K[8] = A * N + P * B + I * C + U * L, K[9] = A * Q + P * O + I * V + U * J, K[10] = A * X + P * Z + I * H + U * h, K[11] = A * Y + P * G + I * E + U * k, A = $[12], P = $[13], I = $[14], U = $[15], K[12] = A * N + P * B + I * C + U * L, K[13] = A * Q + P * O + I * V + U * J, K[14] = A * X + P * Z + I * H + U * h, K[15] = A * Y + P * G + I * E + U * k, K;
};
var CK = function(K, W, $) {
  var N = $[0], Q = $[1], X = $[2], Y, B, O, Z, G, C, V, H, E, L, J, h;
  if (W === K)
    K[12] = W[0] * N + W[4] * Q + W[8] * X + W[12], K[13] = W[1] * N + W[5] * Q + W[9] * X + W[13], K[14] = W[2] * N + W[6] * Q + W[10] * X + W[14], K[15] = W[3] * N + W[7] * Q + W[11] * X + W[15];
  else
    Y = W[0], B = W[1], O = W[2], Z = W[3], G = W[4], C = W[5], V = W[6], H = W[7], E = W[8], L = W[9], J = W[10], h = W[11], K[0] = Y, K[1] = B, K[2] = O, K[3] = Z, K[4] = G, K[5] = C, K[6] = V, K[7] = H, K[8] = E, K[9] = L, K[10] = J, K[11] = h, K[12] = Y * N + G * Q + E * X + W[12], K[13] = B * N + C * Q + L * X + W[13], K[14] = O * N + V * Q + J * X + W[14], K[15] = Z * N + H * Q + h * X + W[15];
  return K;
};
var HK = function(K, W, $) {
  var N = $[0], Q = $[1], X = $[2];
  return K[0] = W[0] * N, K[1] = W[1] * N, K[2] = W[2] * N, K[3] = W[3] * N, K[4] = W[4] * Q, K[5] = W[5] * Q, K[6] = W[6] * Q, K[7] = W[7] * Q, K[8] = W[8] * X, K[9] = W[9] * X, K[10] = W[10] * X, K[11] = W[11] * X, K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15], K;
};
var VK = function(K, W, $, N) {
  var Q = N[0], X = N[1], Y = N[2], B = Math.hypot(Q, X, Y), O, Z, G, C, V, H, E, L, J, h, k, A, P, I, U, g, _, j, M, p, F, q, R, f;
  if (B < D)
    return null;
  if (B = 1 / B, Q *= B, X *= B, Y *= B, O = Math.sin($), Z = Math.cos($), G = 1 - Z, C = W[0], V = W[1], H = W[2], E = W[3], L = W[4], J = W[5], h = W[6], k = W[7], A = W[8], P = W[9], I = W[10], U = W[11], g = Q * Q * G + Z, _ = X * Q * G + Y * O, j = Y * Q * G - X * O, M = Q * X * G - Y * O, p = X * X * G + Z, F = Y * X * G + Q * O, q = Q * Y * G + X * O, R = X * Y * G - Q * O, f = Y * Y * G + Z, K[0] = C * g + L * _ + A * j, K[1] = V * g + J * _ + P * j, K[2] = H * g + h * _ + I * j, K[3] = E * g + k * _ + U * j, K[4] = C * M + L * p + A * F, K[5] = V * M + J * p + P * F, K[6] = H * M + h * p + I * F, K[7] = E * M + k * p + U * F, K[8] = C * q + L * R + A * f, K[9] = V * q + J * R + P * f, K[10] = H * q + h * R + I * f, K[11] = E * q + k * R + U * f, W !== K)
    K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K;
};
var LK = function(K, W, $) {
  var N = Math.sin($), Q = Math.cos($), X = W[4], Y = W[5], B = W[6], O = W[7], Z = W[8], G = W[9], C = W[10], V = W[11];
  if (W !== K)
    K[0] = W[0], K[1] = W[1], K[2] = W[2], K[3] = W[3], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[4] = X * Q + Z * N, K[5] = Y * Q + G * N, K[6] = B * Q + C * N, K[7] = O * Q + V * N, K[8] = Z * Q - X * N, K[9] = G * Q - Y * N, K[10] = C * Q - B * N, K[11] = V * Q - O * N, K;
};
var EK = function(K, W, $) {
  var N = Math.sin($), Q = Math.cos($), X = W[0], Y = W[1], B = W[2], O = W[3], Z = W[8], G = W[9], C = W[10], V = W[11];
  if (W !== K)
    K[4] = W[4], K[5] = W[5], K[6] = W[6], K[7] = W[7], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[0] = X * Q - Z * N, K[1] = Y * Q - G * N, K[2] = B * Q - C * N, K[3] = O * Q - V * N, K[8] = X * N + Z * Q, K[9] = Y * N + G * Q, K[10] = B * N + C * Q, K[11] = O * N + V * Q, K;
};
var hK = function(K, W, $) {
  var N = Math.sin($), Q = Math.cos($), X = W[0], Y = W[1], B = W[2], O = W[3], Z = W[4], G = W[5], C = W[6], V = W[7];
  if (W !== K)
    K[8] = W[8], K[9] = W[9], K[10] = W[10], K[11] = W[11], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[0] = X * Q + Z * N, K[1] = Y * Q + G * N, K[2] = B * Q + C * N, K[3] = O * Q + V * N, K[4] = Z * Q - X * N, K[5] = G * Q - Y * N, K[6] = C * Q - B * N, K[7] = V * Q - O * N, K;
};
var IK = function(K, W) {
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = W[0], K[13] = W[1], K[14] = W[2], K[15] = 1, K;
};
var UK = function(K, W) {
  return K[0] = W[0], K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = W[1], K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = W[2], K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var PK = function(K, W, $) {
  var N = $[0], Q = $[1], X = $[2], Y = Math.hypot(N, Q, X), B, O, Z;
  if (Y < D)
    return null;
  return Y = 1 / Y, N *= Y, Q *= Y, X *= Y, B = Math.sin(W), O = Math.cos(W), Z = 1 - O, K[0] = N * N * Z + O, K[1] = Q * N * Z + X * B, K[2] = X * N * Z - Q * B, K[3] = 0, K[4] = N * Q * Z - X * B, K[5] = Q * Q * Z + O, K[6] = X * Q * Z + N * B, K[7] = 0, K[8] = N * X * Z + Q * B, K[9] = Q * X * Z - N * B, K[10] = X * X * Z + O, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var AK = function(K, W) {
  var $ = Math.sin(W), N = Math.cos(W);
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = N, K[6] = $, K[7] = 0, K[8] = 0, K[9] = -$, K[10] = N, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var DK = function(K, W) {
  var $ = Math.sin(W), N = Math.cos(W);
  return K[0] = N, K[1] = 0, K[2] = -$, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = $, K[9] = 0, K[10] = N, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var kK = function(K, W) {
  var $ = Math.sin(W), N = Math.cos(W);
  return K[0] = N, K[1] = $, K[2] = 0, K[3] = 0, K[4] = -$, K[5] = N, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var J0 = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2], Y = W[3], B = N + N, O = Q + Q, Z = X + X, G = N * B, C = N * O, V = N * Z, H = Q * O, E = Q * Z, L = X * Z, J = Y * B, h = Y * O, k = Y * Z;
  return K[0] = 1 - (H + L), K[1] = C + k, K[2] = V - h, K[3] = 0, K[4] = C - k, K[5] = 1 - (G + L), K[6] = E + J, K[7] = 0, K[8] = V + h, K[9] = E - J, K[10] = 1 - (G + H), K[11] = 0, K[12] = $[0], K[13] = $[1], K[14] = $[2], K[15] = 1, K;
};
var RK = function(K, W) {
  var $ = new S(3), N = -W[0], Q = -W[1], X = -W[2], Y = W[3], B = W[4], O = W[5], Z = W[6], G = W[7], C = N * N + Q * Q + X * X + Y * Y;
  if (C > 0)
    $[0] = (B * Y + G * N + O * X - Z * Q) * 2 / C, $[1] = (O * Y + G * Q + Z * N - B * X) * 2 / C, $[2] = (Z * Y + G * X + B * Q - O * N) * 2 / C;
  else
    $[0] = (B * Y + G * N + O * X - Z * Q) * 2, $[1] = (O * Y + G * Q + Z * N - B * X) * 2, $[2] = (Z * Y + G * X + B * Q - O * N) * 2;
  return J0(K, W, $), K;
};
var TK = function(K, W) {
  return K[0] = W[12], K[1] = W[13], K[2] = W[14], K;
};
var C0 = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[4], Y = W[5], B = W[6], O = W[8], Z = W[9], G = W[10];
  return K[0] = Math.hypot($, N, Q), K[1] = Math.hypot(X, Y, B), K[2] = Math.hypot(O, Z, G), K;
};
var SK = function(K, W) {
  var $ = new S(3);
  C0($, W);
  var N = 1 / $[0], Q = 1 / $[1], X = 1 / $[2], Y = W[0] * N, B = W[1] * Q, O = W[2] * X, Z = W[4] * N, G = W[5] * Q, C = W[6] * X, V = W[8] * N, H = W[9] * Q, E = W[10] * X, L = Y + G + E, J = 0;
  if (L > 0)
    J = Math.sqrt(L + 1) * 2, K[3] = 0.25 * J, K[0] = (C - H) / J, K[1] = (V - O) / J, K[2] = (B - Z) / J;
  else if (Y > G && Y > E)
    J = Math.sqrt(1 + Y - G - E) * 2, K[3] = (C - H) / J, K[0] = 0.25 * J, K[1] = (B + Z) / J, K[2] = (V + O) / J;
  else if (G > E)
    J = Math.sqrt(1 + G - Y - E) * 2, K[3] = (V - O) / J, K[0] = (B + Z) / J, K[1] = 0.25 * J, K[2] = (C + H) / J;
  else
    J = Math.sqrt(1 + E - Y - G) * 2, K[3] = (B - Z) / J, K[0] = (V + O) / J, K[1] = (C + H) / J, K[2] = 0.25 * J;
  return K;
};
var _K = function(K, W, $, N) {
  var Q = W[0], X = W[1], Y = W[2], B = W[3], O = Q + Q, Z = X + X, G = Y + Y, C = Q * O, V = Q * Z, H = Q * G, E = X * Z, L = X * G, J = Y * G, h = B * O, k = B * Z, A = B * G, P = N[0], I = N[1], U = N[2];
  return K[0] = (1 - (E + J)) * P, K[1] = (V + A) * P, K[2] = (H - k) * P, K[3] = 0, K[4] = (V - A) * I, K[5] = (1 - (C + J)) * I, K[6] = (L + h) * I, K[7] = 0, K[8] = (H + k) * U, K[9] = (L - h) * U, K[10] = (1 - (C + E)) * U, K[11] = 0, K[12] = $[0], K[13] = $[1], K[14] = $[2], K[15] = 1, K;
};
var jK = function(K, W, $, N, Q) {
  var X = W[0], Y = W[1], B = W[2], O = W[3], Z = X + X, G = Y + Y, C = B + B, V = X * Z, H = X * G, E = X * C, L = Y * G, J = Y * C, h = B * C, k = O * Z, A = O * G, P = O * C, I = N[0], U = N[1], g = N[2], _ = Q[0], j = Q[1], M = Q[2], p = (1 - (L + h)) * I, F = (H + P) * I, q = (E - A) * I, R = (H - P) * U, f = (1 - (V + h)) * U, l = (J + k) * U, c = (E + A) * g, Y0 = (J - k) * g, B0 = (1 - (V + L)) * g;
  return K[0] = p, K[1] = F, K[2] = q, K[3] = 0, K[4] = R, K[5] = f, K[6] = l, K[7] = 0, K[8] = c, K[9] = Y0, K[10] = B0, K[11] = 0, K[12] = $[0] + _ - (p * _ + R * j + c * M), K[13] = $[1] + j - (F * _ + f * j + Y0 * M), K[14] = $[2] + M - (q * _ + l * j + B0 * M), K[15] = 1, K;
};
var MK = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = $ + $, B = N + N, O = Q + Q, Z = $ * Y, G = N * Y, C = N * B, V = Q * Y, H = Q * B, E = Q * O, L = X * Y, J = X * B, h = X * O;
  return K[0] = 1 - C - E, K[1] = G + h, K[2] = V - J, K[3] = 0, K[4] = G - h, K[5] = 1 - Z - E, K[6] = H + L, K[7] = 0, K[8] = V + J, K[9] = H - L, K[10] = 1 - Z - C, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var gK = function(K, W, $, N, Q, X, Y) {
  var B = 1 / ($ - W), O = 1 / (Q - N), Z = 1 / (X - Y);
  return K[0] = X * 2 * B, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = X * 2 * O, K[6] = 0, K[7] = 0, K[8] = ($ + W) * B, K[9] = (Q + N) * O, K[10] = (Y + X) * Z, K[11] = -1, K[12] = 0, K[13] = 0, K[14] = Y * X * 2 * Z, K[15] = 0, K;
};
var H0 = function(K, W, $, N, Q) {
  var X = 1 / Math.tan(W / 2), Y;
  if (K[0] = X / $, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = X, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = -1, K[12] = 0, K[13] = 0, K[15] = 0, Q != null && Q !== Infinity)
    Y = 1 / (N - Q), K[10] = (Q + N) * Y, K[14] = 2 * Q * N * Y;
  else
    K[10] = -1, K[14] = -2 * N;
  return K;
};
var FK = function(K, W, $, N, Q) {
  var X = 1 / Math.tan(W / 2), Y;
  if (K[0] = X / $, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = X, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = -1, K[12] = 0, K[13] = 0, K[15] = 0, Q != null && Q !== Infinity)
    Y = 1 / (N - Q), K[10] = Q * Y, K[14] = Q * N * Y;
  else
    K[10] = -1, K[14] = -N;
  return K;
};
var qK = function(K, W, $, N) {
  var Q = Math.tan(W.upDegrees * Math.PI / 180), X = Math.tan(W.downDegrees * Math.PI / 180), Y = Math.tan(W.leftDegrees * Math.PI / 180), B = Math.tan(W.rightDegrees * Math.PI / 180), O = 2 / (Y + B), Z = 2 / (Q + X);
  return K[0] = O, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = Z, K[6] = 0, K[7] = 0, K[8] = -((Y - B) * O * 0.5), K[9] = (Q - X) * Z * 0.5, K[10] = N / ($ - N), K[11] = -1, K[12] = 0, K[13] = 0, K[14] = N * $ / ($ - N), K[15] = 0, K;
};
var V0 = function(K, W, $, N, Q, X, Y) {
  var B = 1 / (W - $), O = 1 / (N - Q), Z = 1 / (X - Y);
  return K[0] = -2 * B, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = -2 * O, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 2 * Z, K[11] = 0, K[12] = (W + $) * B, K[13] = (Q + N) * O, K[14] = (Y + X) * Z, K[15] = 1, K;
};
var wK = function(K, W, $, N, Q, X, Y) {
  var B = 1 / (W - $), O = 1 / (N - Q), Z = 1 / (X - Y);
  return K[0] = -2 * B, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = -2 * O, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = Z, K[11] = 0, K[12] = (W + $) * B, K[13] = (Q + N) * O, K[14] = X * Z, K[15] = 1, K;
};
var nK = function(K, W, $, N) {
  var Q, X, Y, B, O, Z, G, C, V, H, E = W[0], L = W[1], J = W[2], h = N[0], k = N[1], A = N[2], P = $[0], I = $[1], U = $[2];
  if (Math.abs(E - P) < D && Math.abs(L - I) < D && Math.abs(J - U) < D)
    return Z0(K);
  if (G = E - P, C = L - I, V = J - U, H = 1 / Math.hypot(G, C, V), G *= H, C *= H, V *= H, Q = k * V - A * C, X = A * G - h * V, Y = h * C - k * G, H = Math.hypot(Q, X, Y), !H)
    Q = 0, X = 0, Y = 0;
  else
    H = 1 / H, Q *= H, X *= H, Y *= H;
  if (B = C * Y - V * X, O = V * Q - G * Y, Z = G * X - C * Q, H = Math.hypot(B, O, Z), !H)
    B = 0, O = 0, Z = 0;
  else
    H = 1 / H, B *= H, O *= H, Z *= H;
  return K[0] = Q, K[1] = B, K[2] = G, K[3] = 0, K[4] = X, K[5] = O, K[6] = C, K[7] = 0, K[8] = Y, K[9] = Z, K[10] = V, K[11] = 0, K[12] = -(Q * E + X * L + Y * J), K[13] = -(B * E + O * L + Z * J), K[14] = -(G * E + C * L + V * J), K[15] = 1, K;
};
var dK = function(K, W, $, N) {
  var Q = W[0], X = W[1], Y = W[2], B = N[0], O = N[1], Z = N[2], G = Q - $[0], C = X - $[1], V = Y - $[2], H = G * G + C * C + V * V;
  if (H > 0)
    H = 1 / Math.sqrt(H), G *= H, C *= H, V *= H;
  var E = O * V - Z * C, L = Z * G - B * V, J = B * C - O * G;
  if (H = E * E + L * L + J * J, H > 0)
    H = 1 / Math.sqrt(H), E *= H, L *= H, J *= H;
  return K[0] = E, K[1] = L, K[2] = J, K[3] = 0, K[4] = C * J - V * L, K[5] = V * E - G * J, K[6] = G * L - C * E, K[7] = 0, K[8] = G, K[9] = C, K[10] = V, K[11] = 0, K[12] = Q, K[13] = X, K[14] = Y, K[15] = 1, K;
};
var vK = function(K) {
  return "mat4(" + K[0] + ", " + K[1] + ", " + K[2] + ", " + K[3] + ", " + K[4] + ", " + K[5] + ", " + K[6] + ", " + K[7] + ", " + K[8] + ", " + K[9] + ", " + K[10] + ", " + K[11] + ", " + K[12] + ", " + K[13] + ", " + K[14] + ", " + K[15] + ")";
};
var iK = function(K) {
  return Math.hypot(K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8], K[9], K[10], K[11], K[12], K[13], K[14], K[15]);
};
var lK = function(K, W, $) {
  return K[0] = W[0] + $[0], K[1] = W[1] + $[1], K[2] = W[2] + $[2], K[3] = W[3] + $[3], K[4] = W[4] + $[4], K[5] = W[5] + $[5], K[6] = W[6] + $[6], K[7] = W[7] + $[7], K[8] = W[8] + $[8], K[9] = W[9] + $[9], K[10] = W[10] + $[10], K[11] = W[11] + $[11], K[12] = W[12] + $[12], K[13] = W[13] + $[13], K[14] = W[14] + $[14], K[15] = W[15] + $[15], K;
};
var L0 = function(K, W, $) {
  return K[0] = W[0] - $[0], K[1] = W[1] - $[1], K[2] = W[2] - $[2], K[3] = W[3] - $[3], K[4] = W[4] - $[4], K[5] = W[5] - $[5], K[6] = W[6] - $[6], K[7] = W[7] - $[7], K[8] = W[8] - $[8], K[9] = W[9] - $[9], K[10] = W[10] - $[10], K[11] = W[11] - $[11], K[12] = W[12] - $[12], K[13] = W[13] - $[13], K[14] = W[14] - $[14], K[15] = W[15] - $[15], K;
};
var cK = function(K, W, $) {
  return K[0] = W[0] * $, K[1] = W[1] * $, K[2] = W[2] * $, K[3] = W[3] * $, K[4] = W[4] * $, K[5] = W[5] * $, K[6] = W[6] * $, K[7] = W[7] * $, K[8] = W[8] * $, K[9] = W[9] * $, K[10] = W[10] * $, K[11] = W[11] * $, K[12] = W[12] * $, K[13] = W[13] * $, K[14] = W[14] * $, K[15] = W[15] * $, K;
};
var yK = function(K, W, $, N) {
  return K[0] = W[0] + $[0] * N, K[1] = W[1] + $[1] * N, K[2] = W[2] + $[2] * N, K[3] = W[3] + $[3] * N, K[4] = W[4] + $[4] * N, K[5] = W[5] + $[5] * N, K[6] = W[6] + $[6] * N, K[7] = W[7] + $[7] * N, K[8] = W[8] + $[8] * N, K[9] = W[9] + $[9] * N, K[10] = W[10] + $[10] * N, K[11] = W[11] + $[11] * N, K[12] = W[12] + $[12] * N, K[13] = W[13] + $[13] * N, K[14] = W[14] + $[14] * N, K[15] = W[15] + $[15] * N, K;
};
var zK = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2] && K[3] === W[3] && K[4] === W[4] && K[5] === W[5] && K[6] === W[6] && K[7] === W[7] && K[8] === W[8] && K[9] === W[9] && K[10] === W[10] && K[11] === W[11] && K[12] === W[12] && K[13] === W[13] && K[14] === W[14] && K[15] === W[15];
};
var rK = function(K, W) {
  var $ = K[0], N = K[1], Q = K[2], X = K[3], Y = K[4], B = K[5], O = K[6], Z = K[7], G = K[8], C = K[9], V = K[10], H = K[11], E = K[12], L = K[13], J = K[14], h = K[15], k = W[0], A = W[1], P = W[2], I = W[3], U = W[4], g = W[5], _ = W[6], j = W[7], M = W[8], p = W[9], F = W[10], q = W[11], R = W[12], f = W[13], l = W[14], c = W[15];
  return Math.abs($ - k) <= D * Math.max(1, Math.abs($), Math.abs(k)) && Math.abs(N - A) <= D * Math.max(1, Math.abs(N), Math.abs(A)) && Math.abs(Q - P) <= D * Math.max(1, Math.abs(Q), Math.abs(P)) && Math.abs(X - I) <= D * Math.max(1, Math.abs(X), Math.abs(I)) && Math.abs(Y - U) <= D * Math.max(1, Math.abs(Y), Math.abs(U)) && Math.abs(B - g) <= D * Math.max(1, Math.abs(B), Math.abs(g)) && Math.abs(O - _) <= D * Math.max(1, Math.abs(O), Math.abs(_)) && Math.abs(Z - j) <= D * Math.max(1, Math.abs(Z), Math.abs(j)) && Math.abs(G - M) <= D * Math.max(1, Math.abs(G), Math.abs(M)) && Math.abs(C - p) <= D * Math.max(1, Math.abs(C), Math.abs(p)) && Math.abs(V - F) <= D * Math.max(1, Math.abs(V), Math.abs(F)) && Math.abs(H - q) <= D * Math.max(1, Math.abs(H), Math.abs(q)) && Math.abs(E - R) <= D * Math.max(1, Math.abs(E), Math.abs(R)) && Math.abs(L - f) <= D * Math.max(1, Math.abs(L), Math.abs(f)) && Math.abs(J - l) <= D * Math.max(1, Math.abs(J), Math.abs(l)) && Math.abs(h - c) <= D * Math.max(1, Math.abs(h), Math.abs(c));
};
var x = function() {
  var K = new S(3);
  if (S != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0;
  return K;
};
var xK = function(K) {
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
var eK = function(K, W) {
  return K[0] = W[0], K[1] = W[1], K[2] = W[2], K;
};
var bK = function(K, W, $, N) {
  return K[0] = W, K[1] = $, K[2] = N, K;
};
var uK = function(K, W, $) {
  return K[0] = W[0] + $[0], K[1] = W[1] + $[1], K[2] = W[2] + $[2], K;
};
var h0 = function(K, W, $) {
  return K[0] = W[0] - $[0], K[1] = W[1] - $[1], K[2] = W[2] - $[2], K;
};
var I0 = function(K, W, $) {
  return K[0] = W[0] * $[0], K[1] = W[1] * $[1], K[2] = W[2] * $[2], K;
};
var U0 = function(K, W, $) {
  return K[0] = W[0] / $[0], K[1] = W[1] / $[1], K[2] = W[2] / $[2], K;
};
var oK = function(K, W) {
  return K[0] = Math.ceil(W[0]), K[1] = Math.ceil(W[1]), K[2] = Math.ceil(W[2]), K;
};
var tK = function(K, W) {
  return K[0] = Math.floor(W[0]), K[1] = Math.floor(W[1]), K[2] = Math.floor(W[2]), K;
};
var aK = function(K, W, $) {
  return K[0] = Math.min(W[0], $[0]), K[1] = Math.min(W[1], $[1]), K[2] = Math.min(W[2], $[2]), K;
};
var KW = function(K, W, $) {
  return K[0] = Math.max(W[0], $[0]), K[1] = Math.max(W[1], $[1]), K[2] = Math.max(W[2], $[2]), K;
};
var WW = function(K, W) {
  return K[0] = Math.round(W[0]), K[1] = Math.round(W[1]), K[2] = Math.round(W[2]), K;
};
var $W = function(K, W, $) {
  return K[0] = W[0] * $, K[1] = W[1] * $, K[2] = W[2] * $, K;
};
var NW = function(K, W, $, N) {
  return K[0] = W[0] + $[0] * N, K[1] = W[1] + $[1] * N, K[2] = W[2] + $[2] * N, K;
};
var P0 = function(K, W) {
  var $ = W[0] - K[0], N = W[1] - K[1], Q = W[2] - K[2];
  return Math.hypot($, N, Q);
};
var A0 = function(K, W) {
  var $ = W[0] - K[0], N = W[1] - K[1], Q = W[2] - K[2];
  return $ * $ + N * N + Q * Q;
};
var D0 = function(K) {
  var W = K[0], $ = K[1], N = K[2];
  return W * W + $ * $ + N * N;
};
var QW = function(K, W) {
  return K[0] = -W[0], K[1] = -W[1], K[2] = -W[2], K;
};
var XW = function(K, W) {
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
var z = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2], Y = $[0], B = $[1], O = $[2];
  return K[0] = Q * O - X * B, K[1] = X * Y - N * O, K[2] = N * B - Q * Y, K;
};
var YW = function(K, W, $, N) {
  var Q = W[0], X = W[1], Y = W[2];
  return K[0] = Q + N * ($[0] - Q), K[1] = X + N * ($[1] - X), K[2] = Y + N * ($[2] - Y), K;
};
var BW = function(K, W, $, N, Q, X) {
  var Y = X * X, B = Y * (2 * X - 3) + 1, O = Y * (X - 2) + X, Z = Y * (X - 1), G = Y * (3 - 2 * X);
  return K[0] = W[0] * B + $[0] * O + N[0] * Z + Q[0] * G, K[1] = W[1] * B + $[1] * O + N[1] * Z + Q[1] * G, K[2] = W[2] * B + $[2] * O + N[2] * Z + Q[2] * G, K;
};
var OW = function(K, W, $, N, Q, X) {
  var Y = 1 - X, B = Y * Y, O = X * X, Z = B * Y, G = 3 * X * B, C = 3 * O * Y, V = O * X;
  return K[0] = W[0] * Z + $[0] * G + N[0] * C + Q[0] * V, K[1] = W[1] * Z + $[1] * G + N[1] * C + Q[1] * V, K[2] = W[2] * Z + $[2] * G + N[2] * C + Q[2] * V, K;
};
var ZW = function(K, W) {
  W = W || 1;
  var $ = d() * 2 * Math.PI, N = d() * 2 - 1, Q = Math.sqrt(1 - N * N) * W;
  return K[0] = Math.cos($) * Q, K[1] = Math.sin($) * Q, K[2] = N * W, K;
};
var GW = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2], Y = $[3] * N + $[7] * Q + $[11] * X + $[15];
  return Y = Y || 1, K[0] = ($[0] * N + $[4] * Q + $[8] * X + $[12]) / Y, K[1] = ($[1] * N + $[5] * Q + $[9] * X + $[13]) / Y, K[2] = ($[2] * N + $[6] * Q + $[10] * X + $[14]) / Y, K;
};
var JW = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2];
  return K[0] = N * $[0] + Q * $[3] + X * $[6], K[1] = N * $[1] + Q * $[4] + X * $[7], K[2] = N * $[2] + Q * $[5] + X * $[8], K;
};
var CW = function(K, W, $) {
  var N = $[0], Q = $[1], X = $[2], Y = $[3], B = W[0], O = W[1], Z = W[2], G = Q * Z - X * O, C = X * B - N * Z, V = N * O - Q * B, H = Q * V - X * C, E = X * G - N * V, L = N * C - Q * G, J = Y * 2;
  return G *= J, C *= J, V *= J, H *= 2, E *= 2, L *= 2, K[0] = B + G + H, K[1] = O + C + E, K[2] = Z + V + L, K;
};
var HW = function(K, W, $, N) {
  var Q = [], X = [];
  return Q[0] = W[0] - $[0], Q[1] = W[1] - $[1], Q[2] = W[2] - $[2], X[0] = Q[0], X[1] = Q[1] * Math.cos(N) - Q[2] * Math.sin(N), X[2] = Q[1] * Math.sin(N) + Q[2] * Math.cos(N), K[0] = X[0] + $[0], K[1] = X[1] + $[1], K[2] = X[2] + $[2], K;
};
var VW = function(K, W, $, N) {
  var Q = [], X = [];
  return Q[0] = W[0] - $[0], Q[1] = W[1] - $[1], Q[2] = W[2] - $[2], X[0] = Q[2] * Math.sin(N) + Q[0] * Math.cos(N), X[1] = Q[1], X[2] = Q[2] * Math.cos(N) - Q[0] * Math.sin(N), K[0] = X[0] + $[0], K[1] = X[1] + $[1], K[2] = X[2] + $[2], K;
};
var LW = function(K, W, $, N) {
  var Q = [], X = [];
  return Q[0] = W[0] - $[0], Q[1] = W[1] - $[1], Q[2] = W[2] - $[2], X[0] = Q[0] * Math.cos(N) - Q[1] * Math.sin(N), X[1] = Q[0] * Math.sin(N) + Q[1] * Math.cos(N), X[2] = Q[2], K[0] = X[0] + $[0], K[1] = X[1] + $[1], K[2] = X[2] + $[2], K;
};
var EW = function(K, W) {
  var $ = K[0], N = K[1], Q = K[2], X = W[0], Y = W[1], B = W[2], O = Math.sqrt($ * $ + N * N + Q * Q), Z = Math.sqrt(X * X + Y * Y + B * B), G = O * Z, C = G && b(K, W) / G;
  return Math.acos(Math.min(Math.max(C, -1), 1));
};
var hW = function(K) {
  return K[0] = 0, K[1] = 0, K[2] = 0, K;
};
var IW = function(K) {
  return "vec3(" + K[0] + ", " + K[1] + ", " + K[2] + ")";
};
var UW = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2];
};
var PW = function(K, W) {
  var $ = K[0], N = K[1], Q = K[2], X = W[0], Y = W[1], B = W[2];
  return Math.abs($ - X) <= D * Math.max(1, Math.abs($), Math.abs(X)) && Math.abs(N - Y) <= D * Math.max(1, Math.abs(N), Math.abs(Y)) && Math.abs(Q - B) <= D * Math.max(1, Math.abs(Q), Math.abs(B));
};
var jW = function() {
  var K = new S(4);
  if (S != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0, K[3] = 0;
  return K;
};
var k0 = function(K) {
  var W = new S(4);
  return W[0] = K[0], W[1] = K[1], W[2] = K[2], W[3] = K[3], W;
};
var R0 = function(K, W, $, N) {
  var Q = new S(4);
  return Q[0] = K, Q[1] = W, Q[2] = $, Q[3] = N, Q;
};
var T0 = function(K, W) {
  return K[0] = W[0], K[1] = W[1], K[2] = W[2], K[3] = W[3], K;
};
var S0 = function(K, W, $, N, Q) {
  return K[0] = W, K[1] = $, K[2] = N, K[3] = Q, K;
};
var _0 = function(K, W, $) {
  return K[0] = W[0] + $[0], K[1] = W[1] + $[1], K[2] = W[2] + $[2], K[3] = W[3] + $[3], K;
};
var j0 = function(K, W, $) {
  return K[0] = W[0] * $, K[1] = W[1] * $, K[2] = W[2] * $, K[3] = W[3] * $, K;
};
var M0 = function(K) {
  var W = K[0], $ = K[1], N = K[2], Q = K[3];
  return Math.hypot(W, $, N, Q);
};
var g0 = function(K) {
  var W = K[0], $ = K[1], N = K[2], Q = K[3];
  return W * W + $ * $ + N * N + Q * Q;
};
var p0 = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = $ * $ + N * N + Q * Q + X * X;
  if (Y > 0)
    Y = 1 / Math.sqrt(Y);
  return K[0] = $ * Y, K[1] = N * Y, K[2] = Q * Y, K[3] = X * Y, K;
};
var F0 = function(K, W) {
  return K[0] * W[0] + K[1] * W[1] + K[2] * W[2] + K[3] * W[3];
};
var q0 = function(K, W, $, N) {
  var Q = W[0], X = W[1], Y = W[2], B = W[3];
  return K[0] = Q + N * ($[0] - Q), K[1] = X + N * ($[1] - X), K[2] = Y + N * ($[2] - Y), K[3] = B + N * ($[3] - B), K;
};
var f0 = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2] && K[3] === W[3];
};
var w0 = function(K, W) {
  var $ = K[0], N = K[1], Q = K[2], X = K[3], Y = W[0], B = W[1], O = W[2], Z = W[3];
  return Math.abs($ - Y) <= D * Math.max(1, Math.abs($), Math.abs(Y)) && Math.abs(N - B) <= D * Math.max(1, Math.abs(N), Math.abs(B)) && Math.abs(Q - O) <= D * Math.max(1, Math.abs(Q), Math.abs(O)) && Math.abs(X - Z) <= D * Math.max(1, Math.abs(X), Math.abs(Z));
};
var W0 = function() {
  var K = new S(4);
  if (S != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0;
  return K[3] = 1, K;
};
var gW = function(K) {
  return K[0] = 0, K[1] = 0, K[2] = 0, K[3] = 1, K;
};
var n0 = function(K, W, $) {
  $ = $ * 0.5;
  var N = Math.sin($);
  return K[0] = N * W[0], K[1] = N * W[1], K[2] = N * W[2], K[3] = Math.cos($), K;
};
var pW = function(K, W) {
  var $ = Math.acos(W[3]) * 2, N = Math.sin($ / 2);
  if (N > D)
    K[0] = W[0] / N, K[1] = W[1] / N, K[2] = W[2] / N;
  else
    K[0] = 1, K[1] = 0, K[2] = 0;
  return $;
};
var FW = function(K, W) {
  var $ = y0(K, W);
  return Math.acos(2 * $ * $ - 1);
};
var d0 = function(K, W, $) {
  var N = W[0], Q = W[1], X = W[2], Y = W[3], B = $[0], O = $[1], Z = $[2], G = $[3];
  return K[0] = N * G + Y * B + Q * Z - X * O, K[1] = Q * G + Y * O + X * B - N * Z, K[2] = X * G + Y * Z + N * O - Q * B, K[3] = Y * G - N * B - Q * O - X * Z, K;
};
var qW = function(K, W, $) {
  $ *= 0.5;
  var N = W[0], Q = W[1], X = W[2], Y = W[3], B = Math.sin($), O = Math.cos($);
  return K[0] = N * O + Y * B, K[1] = Q * O + X * B, K[2] = X * O - Q * B, K[3] = Y * O - N * B, K;
};
var fW = function(K, W, $) {
  $ *= 0.5;
  var N = W[0], Q = W[1], X = W[2], Y = W[3], B = Math.sin($), O = Math.cos($);
  return K[0] = N * O - X * B, K[1] = Q * O + Y * B, K[2] = X * O + N * B, K[3] = Y * O - Q * B, K;
};
var wW = function(K, W, $) {
  $ *= 0.5;
  var N = W[0], Q = W[1], X = W[2], Y = W[3], B = Math.sin($), O = Math.cos($);
  return K[0] = N * O + Q * B, K[1] = Q * O - N * B, K[2] = X * O + Y * B, K[3] = Y * O - X * B, K;
};
var nW = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2];
  return K[0] = $, K[1] = N, K[2] = Q, K[3] = Math.sqrt(Math.abs(1 - $ * $ - N * N - Q * Q)), K;
};
var v0 = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = Math.sqrt($ * $ + N * N + Q * Q), B = Math.exp(X), O = Y > 0 ? B * Math.sin(Y) / Y : 0;
  return K[0] = $ * O, K[1] = N * O, K[2] = Q * O, K[3] = B * Math.cos(Y), K;
};
var i0 = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = Math.sqrt($ * $ + N * N + Q * Q), B = Y > 0 ? Math.atan2(Y, X) / Y : 0;
  return K[0] = $ * B, K[1] = N * B, K[2] = Q * B, K[3] = 0.5 * Math.log($ * $ + N * N + Q * Q + X * X), K;
};
var dW = function(K, W, $) {
  return i0(K, W), c0(K, K, $), v0(K, K), K;
};
var u = function(K, W, $, N) {
  var Q = W[0], X = W[1], Y = W[2], B = W[3], O = $[0], Z = $[1], G = $[2], C = $[3], V, H, E, L, J;
  if (H = Q * O + X * Z + Y * G + B * C, H < 0)
    H = -H, O = -O, Z = -Z, G = -G, C = -C;
  if (1 - H > D)
    V = Math.acos(H), E = Math.sin(V), L = Math.sin((1 - N) * V) / E, J = Math.sin(N * V) / E;
  else
    L = 1 - N, J = N;
  return K[0] = L * Q + J * O, K[1] = L * X + J * Z, K[2] = L * Y + J * G, K[3] = L * B + J * C, K;
};
var vW = function(K) {
  var W = d(), $ = d(), N = d(), Q = Math.sqrt(1 - W), X = Math.sqrt(W);
  return K[0] = Q * Math.sin(2 * Math.PI * $), K[1] = Q * Math.cos(2 * Math.PI * $), K[2] = X * Math.sin(2 * Math.PI * N), K[3] = X * Math.cos(2 * Math.PI * N), K;
};
var iW = function(K, W) {
  var $ = W[0], N = W[1], Q = W[2], X = W[3], Y = $ * $ + N * N + Q * Q + X * X, B = Y ? 1 / Y : 0;
  return K[0] = -$ * B, K[1] = -N * B, K[2] = -Q * B, K[3] = X * B, K;
};
var lW = function(K, W) {
  return K[0] = -W[0], K[1] = -W[1], K[2] = -W[2], K[3] = W[3], K;
};
var l0 = function(K, W) {
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
var cW = function(K, W, $, N) {
  var Q = 0.5 * Math.PI / 180;
  W *= Q, $ *= Q, N *= Q;
  var X = Math.sin(W), Y = Math.cos(W), B = Math.sin($), O = Math.cos($), Z = Math.sin(N), G = Math.cos(N);
  return K[0] = X * O * G - Y * B * Z, K[1] = Y * B * G + X * O * Z, K[2] = Y * O * Z - X * B * G, K[3] = Y * O * G + X * B * Z, K;
};
var yW = function(K) {
  return "quat(" + K[0] + ", " + K[1] + ", " + K[2] + ", " + K[3] + ")";
};
var KK = Object.defineProperty;
var t = (K, W) => {
  for (var $ in W)
    KK(K, $, { get: W[$], enumerable: true, configurable: true, set: (N) => W[$] = () => N });
};
var D = 0.000001;
var S = typeof Float32Array !== "undefined" ? Float32Array : Array;
var d = Math.random;
var J$ = Math.PI / 180;
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
    return OK;
  }
}, translate: () => {
  {
    return CK;
  }
}, targetTo: () => {
  {
    return dK;
  }
}, subtract: () => {
  {
    return L0;
  }
}, sub: () => {
  {
    return mK;
  }
}, str: () => {
  {
    return vK;
  }
}, set: () => {
  {
    return BK;
  }
}, scale: () => {
  {
    return HK;
  }
}, rotateZ: () => {
  {
    return hK;
  }
}, rotateY: () => {
  {
    return EK;
  }
}, rotateX: () => {
  {
    return LK;
  }
}, rotate: () => {
  {
    return VK;
  }
}, perspectiveZO: () => {
  {
    return FK;
  }
}, perspectiveNO: () => {
  {
    return H0;
  }
}, perspectiveFromFieldOfView: () => {
  {
    return qK;
  }
}, perspective: () => {
  {
    return pK;
  }
}, orthoZO: () => {
  {
    return wK;
  }
}, orthoNO: () => {
  {
    return V0;
  }
}, ortho: () => {
  {
    return fK;
  }
}, multiplyScalarAndAdd: () => {
  {
    return yK;
  }
}, multiplyScalar: () => {
  {
    return cK;
  }
}, multiply: () => {
  {
    return G0;
  }
}, mul: () => {
  {
    return sK;
  }
}, lookAt: () => {
  {
    return nK;
  }
}, invert: () => {
  {
    return ZK;
  }
}, identity: () => {
  {
    return Z0;
  }
}, getTranslation: () => {
  {
    return TK;
  }
}, getScaling: () => {
  {
    return C0;
  }
}, getRotation: () => {
  {
    return SK;
  }
}, frustum: () => {
  {
    return gK;
  }
}, fromZRotation: () => {
  {
    return kK;
  }
}, fromYRotation: () => {
  {
    return DK;
  }
}, fromXRotation: () => {
  {
    return AK;
  }
}, fromValues: () => {
  {
    return YK;
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
    return jK;
  }
}, fromRotationTranslationScale: () => {
  {
    return _K;
  }
}, fromRotationTranslation: () => {
  {
    return J0;
  }
}, fromRotation: () => {
  {
    return PK;
  }
}, fromQuat2: () => {
  {
    return RK;
  }
}, fromQuat: () => {
  {
    return MK;
  }
}, frob: () => {
  {
    return iK;
  }
}, exactEquals: () => {
  {
    return zK;
  }
}, equals: () => {
  {
    return rK;
  }
}, determinant: () => {
  {
    return JK;
  }
}, create: () => {
  {
    return NK;
  }
}, copy: () => {
  {
    return XK;
  }
}, clone: () => {
  {
    return QK;
  }
}, adjoint: () => {
  {
    return GK;
  }
}, add: () => {
  {
    return lK;
  }
} });
var pK = H0;
var fK = V0;
var sK = G0;
var mK = L0;
var s = {};
t(s, { str: () => {
  {
    return yW;
  }
}, squaredLength: () => {
  {
    return r0;
  }
}, sqrLen: () => {
  {
    return oW;
  }
}, sqlerp: () => {
  {
    return W$;
  }
}, slerp: () => {
  {
    return u;
  }
}, setAxisAngle: () => {
  {
    return n0;
  }
}, setAxes: () => {
  {
    return $$;
  }
}, set: () => {
  {
    return mW;
  }
}, scale: () => {
  {
    return c0;
  }
}, rotationTo: () => {
  {
    return K$;
  }
}, rotateZ: () => {
  {
    return wW;
  }
}, rotateY: () => {
  {
    return fW;
  }
}, rotateX: () => {
  {
    return qW;
  }
}, random: () => {
  {
    return vW;
  }
}, pow: () => {
  {
    return dW;
  }
}, normalize: () => {
  {
    return $0;
  }
}, multiply: () => {
  {
    return d0;
  }
}, mul: () => {
  {
    return eW;
  }
}, ln: () => {
  {
    return i0;
  }
}, lerp: () => {
  {
    return bW;
  }
}, length: () => {
  {
    return z0;
  }
}, len: () => {
  {
    return uW;
  }
}, invert: () => {
  {
    return iW;
  }
}, identity: () => {
  {
    return gW;
  }
}, getAxisAngle: () => {
  {
    return pW;
  }
}, getAngle: () => {
  {
    return FW;
  }
}, fromValues: () => {
  {
    return rW;
  }
}, fromMat3: () => {
  {
    return l0;
  }
}, fromEuler: () => {
  {
    return cW;
  }
}, exp: () => {
  {
    return v0;
  }
}, exactEquals: () => {
  {
    return tW;
  }
}, equals: () => {
  {
    return aW;
  }
}, dot: () => {
  {
    return y0;
  }
}, create: () => {
  {
    return W0;
  }
}, copy: () => {
  {
    return sW;
  }
}, conjugate: () => {
  {
    return lW;
  }
}, clone: () => {
  {
    return zW;
  }
}, calculateW: () => {
  {
    return nW;
  }
}, add: () => {
  {
    return xW;
  }
} });
var r = {};
t(r, { zero: () => {
  {
    return hW;
  }
}, transformQuat: () => {
  {
    return CW;
  }
}, transformMat4: () => {
  {
    return GW;
  }
}, transformMat3: () => {
  {
    return JW;
  }
}, subtract: () => {
  {
    return h0;
  }
}, sub: () => {
  {
    return AW;
  }
}, str: () => {
  {
    return IW;
  }
}, squaredLength: () => {
  {
    return D0;
  }
}, squaredDistance: () => {
  {
    return A0;
  }
}, sqrLen: () => {
  {
    return SW;
  }
}, sqrDist: () => {
  {
    return TW;
  }
}, set: () => {
  {
    return bK;
  }
}, scaleAndAdd: () => {
  {
    return NW;
  }
}, scale: () => {
  {
    return $W;
  }
}, round: () => {
  {
    return WW;
  }
}, rotateZ: () => {
  {
    return LW;
  }
}, rotateY: () => {
  {
    return VW;
  }
}, rotateX: () => {
  {
    return HW;
  }
}, random: () => {
  {
    return ZW;
  }
}, normalize: () => {
  {
    return a;
  }
}, negate: () => {
  {
    return QW;
  }
}, multiply: () => {
  {
    return I0;
  }
}, mul: () => {
  {
    return DW;
  }
}, min: () => {
  {
    return aK;
  }
}, max: () => {
  {
    return KW;
  }
}, lerp: () => {
  {
    return YW;
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
    return XW;
  }
}, hermite: () => {
  {
    return BW;
  }
}, fromValues: () => {
  {
    return e;
  }
}, forEach: () => {
  {
    return _W;
  }
}, floor: () => {
  {
    return tK;
  }
}, exactEquals: () => {
  {
    return UW;
  }
}, equals: () => {
  {
    return PW;
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
    return kW;
  }
}, distance: () => {
  {
    return P0;
  }
}, dist: () => {
  {
    return RW;
  }
}, cross: () => {
  {
    return z;
  }
}, create: () => {
  {
    return x;
  }
}, copy: () => {
  {
    return eK;
  }
}, clone: () => {
  {
    return xK;
  }
}, ceil: () => {
  {
    return oK;
  }
}, bezier: () => {
  {
    return OW;
  }
}, angle: () => {
  {
    return EW;
  }
}, add: () => {
  {
    return uK;
  }
} });
var AW = h0;
var DW = I0;
var kW = U0;
var RW = P0;
var TW = A0;
var K0 = E0;
var SW = D0;
var _W = function() {
  var K = x();
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
var C$ = function() {
  var K = jW();
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
var zW = k0;
var rW = R0;
var sW = T0;
var mW = S0;
var xW = _0;
var eW = d0;
var c0 = j0;
var y0 = F0;
var bW = q0;
var z0 = M0;
var uW = z0;
var r0 = g0;
var oW = r0;
var $0 = p0;
var tW = f0;
var aW = w0;
var K$ = function() {
  var K = x(), W = e(1, 0, 0), $ = e(0, 1, 0);
  return function(N, Q, X) {
    var Y = b(Q, X);
    if (Y < -0.999999) {
      if (z(K, W, Q), K0(K) < 0.000001)
        z(K, $, Q);
      return a(K, K), n0(N, K, Math.PI), N;
    } else if (Y > 0.999999)
      return N[0] = 0, N[1] = 0, N[2] = 0, N[3] = 1, N;
    else
      return z(K, Q, X), N[0] = K[0], N[1] = K[1], N[2] = K[2], N[3] = 1 + Y, $0(N, N);
  };
}();
var W$ = function() {
  var K = W0(), W = W0();
  return function($, N, Q, X, Y, B) {
    return u(K, N, Y, B), u(W, Q, X, B), u($, K, W, 2 * B * (1 - B)), $;
  };
}();
var $$ = function() {
  var K = O0();
  return function(W, $, N, Q) {
    return K[0] = N[0], K[3] = N[1], K[6] = N[2], K[1] = Q[0], K[4] = Q[1], K[7] = Q[2], K[2] = -$[0], K[5] = -$[1], K[8] = -$[2], $0(W, l0(W, K));
  };
}();
var N$ = Math.PI / 90;
var N0 = [0, 0, 0];
var s0 = T.create();
var m0 = T.create();
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
    return T.perspective(this.#K, K * N$, W, $, N), this;
  }
  ortho(K, W, $, N, Q, X) {
    return T.ortho(this.#K, K, W, $, N, Q, X), this;
  }
  combine(K, W, $ = 0.5) {
    return T.multiplyScalar(s0, K.getMatrix(), 1 - $), T.multiplyScalar(m0, W.getMatrix(), $), T.add(this.#K, s0, m0), this;
  }
  static getMoveVector(K, W, $, N) {
    const Q = N0;
    if (Q[0] = K, Q[1] = W, Q[2] = $, N)
      T.getRotation(o, N.getMatrix()), s.invert(o, o), r.transformQuat(Q, Q, o);
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

class x0 {
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

class e0 {
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

class b0 extends e0 {
  constructor() {
    super((K, W) => {
      if (!K)
        return new x0(W, ($) => $.valueOf(), ($, N) => $.setValue(N));
      return K.element = W, K;
    });
  }
}
var Q$ = new b0;

class v {
  w;
  q;
  #K = 0;
  #W;
  constructor(K = 0, W, $ = Q$) {
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
class o0 {
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

class Q0 extends o0 {
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
var X0 = function(K, W) {
  if (K) {
    const $ = K.length.valueOf();
    for (let N = 0;N < $; N++) {
      const Q = K.at(N);
      if (Q !== undefined && W(Q, N))
        return true;
    }
  }
  return false;
};

class t0 {
  #K = w.create().setPosition(0, 0, 0);
  #W = new Set;
  #$ = [0, 0, 0];
  position = [0, 0, 0];
  blockers;
  constructor({ blockers: K } = {}, W) {
    if (this.blockers = K, W)
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
    const Q = w.getMoveVector(K, W, $, N), X = X0(this.blockers, (Y) => Y.isBlocked(i.toVector(this.position[0] + Q[0], this.position[1] + Q[1], this.position[2] + Q[2], this.#$), this.position));
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
    const N = X0(this.blockers, (Q) => Q.isBlocked(i.toVector(K, W, $, this.#$), this.position));
    if (!N) {
      const [Q, X, Y] = this.#K.getPosition();
      if (Q !== K || X !== W || Y !== $) {
        const B = K - Q, O = W - X, Z = $ - Y;
        this.#K.setPosition(K, W, $), this.changedPosition(B, O, Z);
      }
    }
    return N ? n.BLOCKED : n.MOVED;
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
var B$ = 1;
var O$ = 1;

class a0 {
  K;
  #K = w.create();
  #W = w.create();
  #$ = w.create();
  #N = [0, 0];
  perspective;
  zoom;
  constructor(K) {
    this.onChange = K;
    this.perspective = new v(B$, K), this.zoom = new v(O$, (W) => {
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
