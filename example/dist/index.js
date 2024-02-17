// /Users/vincent/dok-matrix/example/node_modules/dok-matrix/dist/index.js
var O0 = function() {
  var K = new _(9);
  if (_ != Float32Array)
    K[1] = 0, K[2] = 0, K[3] = 0, K[5] = 0, K[6] = 0, K[7] = 0;
  return K[0] = 1, K[4] = 1, K[8] = 1, K;
};
var YK = function() {
  var K = new _(16);
  if (_ != Float32Array)
    K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0;
  return K[0] = 1, K[5] = 1, K[10] = 1, K[15] = 1, K;
};
var $K = function(K) {
  var N = new _(16);
  return N[0] = K[0], N[1] = K[1], N[2] = K[2], N[3] = K[3], N[4] = K[4], N[5] = K[5], N[6] = K[6], N[7] = K[7], N[8] = K[8], N[9] = K[9], N[10] = K[10], N[11] = K[11], N[12] = K[12], N[13] = K[13], N[14] = K[14], N[15] = K[15], N;
};
var QK = function(K, N) {
  return K[0] = N[0], K[1] = N[1], K[2] = N[2], K[3] = N[3], K[4] = N[4], K[5] = N[5], K[6] = N[6], K[7] = N[7], K[8] = N[8], K[9] = N[9], K[10] = N[10], K[11] = N[11], K[12] = N[12], K[13] = N[13], K[14] = N[14], K[15] = N[15], K;
};
var XK = function(K, N, W, Y, $, Q, X, B, O, Z, G, H, V, C, L, E) {
  var J = new _(16);
  return J[0] = K, J[1] = N, J[2] = W, J[3] = Y, J[4] = $, J[5] = Q, J[6] = X, J[7] = B, J[8] = O, J[9] = Z, J[10] = G, J[11] = H, J[12] = V, J[13] = C, J[14] = L, J[15] = E, J;
};
var BK = function(K, N, W, Y, $, Q, X, B, O, Z, G, H, V, C, L, E, J) {
  return K[0] = N, K[1] = W, K[2] = Y, K[3] = $, K[4] = Q, K[5] = X, K[6] = B, K[7] = O, K[8] = Z, K[9] = G, K[10] = H, K[11] = V, K[12] = C, K[13] = L, K[14] = E, K[15] = J, K;
};
var Z0 = function(K) {
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var OK = function(K, N) {
  if (K === N) {
    var W = N[1], Y = N[2], $ = N[3], Q = N[6], X = N[7], B = N[11];
    K[1] = N[4], K[2] = N[8], K[3] = N[12], K[4] = W, K[6] = N[9], K[7] = N[13], K[8] = Y, K[9] = Q, K[11] = N[14], K[12] = $, K[13] = X, K[14] = B;
  } else
    K[0] = N[0], K[1] = N[4], K[2] = N[8], K[3] = N[12], K[4] = N[1], K[5] = N[5], K[6] = N[9], K[7] = N[13], K[8] = N[2], K[9] = N[6], K[10] = N[10], K[11] = N[14], K[12] = N[3], K[13] = N[7], K[14] = N[11], K[15] = N[15];
  return K;
};
var ZK = function(K, N) {
  var W = N[0], Y = N[1], $ = N[2], Q = N[3], X = N[4], B = N[5], O = N[6], Z = N[7], G = N[8], H = N[9], V = N[10], C = N[11], L = N[12], E = N[13], J = N[14], I = N[15], R = W * B - Y * X, A = W * O - $ * X, P = W * Z - Q * X, U = Y * O - $ * B, h = Y * Z - Q * B, g = $ * Z - Q * O, k = G * E - H * L, j = G * J - V * L, M = G * I - C * L, p = H * J - V * E, F = H * I - C * E, q = V * I - C * J, T = R * q - A * F + P * p + U * M - h * j + g * k;
  if (!T)
    return null;
  return T = 1 / T, K[0] = (B * q - O * F + Z * p) * T, K[1] = ($ * F - Y * q - Q * p) * T, K[2] = (E * g - J * h + I * U) * T, K[3] = (V * h - H * g - C * U) * T, K[4] = (O * M - X * q - Z * j) * T, K[5] = (W * q - $ * M + Q * j) * T, K[6] = (J * P - L * g - I * A) * T, K[7] = (G * g - V * P + C * A) * T, K[8] = (X * F - B * M + Z * k) * T, K[9] = (Y * M - W * F - Q * k) * T, K[10] = (L * h - E * P + I * R) * T, K[11] = (H * P - G * h - C * R) * T, K[12] = (B * j - X * p - O * k) * T, K[13] = (W * p - Y * j + $ * k) * T, K[14] = (E * A - L * U - J * R) * T, K[15] = (G * U - H * A + V * R) * T, K;
};
var GK = function(K, N) {
  var W = N[0], Y = N[1], $ = N[2], Q = N[3], X = N[4], B = N[5], O = N[6], Z = N[7], G = N[8], H = N[9], V = N[10], C = N[11], L = N[12], E = N[13], J = N[14], I = N[15];
  return K[0] = B * (V * I - C * J) - H * (O * I - Z * J) + E * (O * C - Z * V), K[1] = -(Y * (V * I - C * J) - H * ($ * I - Q * J) + E * ($ * C - Q * V)), K[2] = Y * (O * I - Z * J) - B * ($ * I - Q * J) + E * ($ * Z - Q * O), K[3] = -(Y * (O * C - Z * V) - B * ($ * C - Q * V) + H * ($ * Z - Q * O)), K[4] = -(X * (V * I - C * J) - G * (O * I - Z * J) + L * (O * C - Z * V)), K[5] = W * (V * I - C * J) - G * ($ * I - Q * J) + L * ($ * C - Q * V), K[6] = -(W * (O * I - Z * J) - X * ($ * I - Q * J) + L * ($ * Z - Q * O)), K[7] = W * (O * C - Z * V) - X * ($ * C - Q * V) + G * ($ * Z - Q * O), K[8] = X * (H * I - C * E) - G * (B * I - Z * E) + L * (B * C - Z * H), K[9] = -(W * (H * I - C * E) - G * (Y * I - Q * E) + L * (Y * C - Q * H)), K[10] = W * (B * I - Z * E) - X * (Y * I - Q * E) + L * (Y * Z - Q * B), K[11] = -(W * (B * C - Z * H) - X * (Y * C - Q * H) + G * (Y * Z - Q * B)), K[12] = -(X * (H * J - V * E) - G * (B * J - O * E) + L * (B * V - O * H)), K[13] = W * (H * J - V * E) - G * (Y * J - $ * E) + L * (Y * V - $ * H), K[14] = -(W * (B * J - O * E) - X * (Y * J - $ * E) + L * (Y * O - $ * B)), K[15] = W * (B * V - O * H) - X * (Y * V - $ * H) + G * (Y * O - $ * B), K;
};
var JK = function(K) {
  var N = K[0], W = K[1], Y = K[2], $ = K[3], Q = K[4], X = K[5], B = K[6], O = K[7], Z = K[8], G = K[9], H = K[10], V = K[11], C = K[12], L = K[13], E = K[14], J = K[15], I = N * X - W * Q, R = N * B - Y * Q, A = N * O - $ * Q, P = W * B - Y * X, U = W * O - $ * X, h = Y * O - $ * B, g = Z * L - G * C, k = Z * E - H * C, j = Z * J - V * C, M = G * E - H * L, p = G * J - V * L, F = H * J - V * E;
  return I * F - R * p + A * M + P * j - U * k + h * g;
};
var G0 = function(K, N, W) {
  var Y = N[0], $ = N[1], Q = N[2], X = N[3], B = N[4], O = N[5], Z = N[6], G = N[7], H = N[8], V = N[9], C = N[10], L = N[11], E = N[12], J = N[13], I = N[14], R = N[15], A = W[0], P = W[1], U = W[2], h = W[3];
  return K[0] = A * Y + P * B + U * H + h * E, K[1] = A * $ + P * O + U * V + h * J, K[2] = A * Q + P * Z + U * C + h * I, K[3] = A * X + P * G + U * L + h * R, A = W[4], P = W[5], U = W[6], h = W[7], K[4] = A * Y + P * B + U * H + h * E, K[5] = A * $ + P * O + U * V + h * J, K[6] = A * Q + P * Z + U * C + h * I, K[7] = A * X + P * G + U * L + h * R, A = W[8], P = W[9], U = W[10], h = W[11], K[8] = A * Y + P * B + U * H + h * E, K[9] = A * $ + P * O + U * V + h * J, K[10] = A * Q + P * Z + U * C + h * I, K[11] = A * X + P * G + U * L + h * R, A = W[12], P = W[13], U = W[14], h = W[15], K[12] = A * Y + P * B + U * H + h * E, K[13] = A * $ + P * O + U * V + h * J, K[14] = A * Q + P * Z + U * C + h * I, K[15] = A * X + P * G + U * L + h * R, K;
};
var HK = function(K, N, W) {
  var Y = W[0], $ = W[1], Q = W[2], X, B, O, Z, G, H, V, C, L, E, J, I;
  if (N === K)
    K[12] = N[0] * Y + N[4] * $ + N[8] * Q + N[12], K[13] = N[1] * Y + N[5] * $ + N[9] * Q + N[13], K[14] = N[2] * Y + N[6] * $ + N[10] * Q + N[14], K[15] = N[3] * Y + N[7] * $ + N[11] * Q + N[15];
  else
    X = N[0], B = N[1], O = N[2], Z = N[3], G = N[4], H = N[5], V = N[6], C = N[7], L = N[8], E = N[9], J = N[10], I = N[11], K[0] = X, K[1] = B, K[2] = O, K[3] = Z, K[4] = G, K[5] = H, K[6] = V, K[7] = C, K[8] = L, K[9] = E, K[10] = J, K[11] = I, K[12] = X * Y + G * $ + L * Q + N[12], K[13] = B * Y + H * $ + E * Q + N[13], K[14] = O * Y + V * $ + J * Q + N[14], K[15] = Z * Y + C * $ + I * Q + N[15];
  return K;
};
var CK = function(K, N, W) {
  var Y = W[0], $ = W[1], Q = W[2];
  return K[0] = N[0] * Y, K[1] = N[1] * Y, K[2] = N[2] * Y, K[3] = N[3] * Y, K[4] = N[4] * $, K[5] = N[5] * $, K[6] = N[6] * $, K[7] = N[7] * $, K[8] = N[8] * Q, K[9] = N[9] * Q, K[10] = N[10] * Q, K[11] = N[11] * Q, K[12] = N[12], K[13] = N[13], K[14] = N[14], K[15] = N[15], K;
};
var VK = function(K, N, W, Y) {
  var $ = Y[0], Q = Y[1], X = Y[2], B = Math.hypot($, Q, X), O, Z, G, H, V, C, L, E, J, I, R, A, P, U, h, g, k, j, M, p, F, q, T, f;
  if (B < D)
    return null;
  if (B = 1 / B, $ *= B, Q *= B, X *= B, O = Math.sin(W), Z = Math.cos(W), G = 1 - Z, H = N[0], V = N[1], C = N[2], L = N[3], E = N[4], J = N[5], I = N[6], R = N[7], A = N[8], P = N[9], U = N[10], h = N[11], g = $ * $ * G + Z, k = Q * $ * G + X * O, j = X * $ * G - Q * O, M = $ * Q * G - X * O, p = Q * Q * G + Z, F = X * Q * G + $ * O, q = $ * X * G + Q * O, T = Q * X * G - $ * O, f = X * X * G + Z, K[0] = H * g + E * k + A * j, K[1] = V * g + J * k + P * j, K[2] = C * g + I * k + U * j, K[3] = L * g + R * k + h * j, K[4] = H * M + E * p + A * F, K[5] = V * M + J * p + P * F, K[6] = C * M + I * p + U * F, K[7] = L * M + R * p + h * F, K[8] = H * q + E * T + A * f, K[9] = V * q + J * T + P * f, K[10] = C * q + I * T + U * f, K[11] = L * q + R * T + h * f, N !== K)
    K[12] = N[12], K[13] = N[13], K[14] = N[14], K[15] = N[15];
  return K;
};
var EK = function(K, N, W) {
  var Y = Math.sin(W), $ = Math.cos(W), Q = N[4], X = N[5], B = N[6], O = N[7], Z = N[8], G = N[9], H = N[10], V = N[11];
  if (N !== K)
    K[0] = N[0], K[1] = N[1], K[2] = N[2], K[3] = N[3], K[12] = N[12], K[13] = N[13], K[14] = N[14], K[15] = N[15];
  return K[4] = Q * $ + Z * Y, K[5] = X * $ + G * Y, K[6] = B * $ + H * Y, K[7] = O * $ + V * Y, K[8] = Z * $ - Q * Y, K[9] = G * $ - X * Y, K[10] = H * $ - B * Y, K[11] = V * $ - O * Y, K;
};
var LK = function(K, N, W) {
  var Y = Math.sin(W), $ = Math.cos(W), Q = N[0], X = N[1], B = N[2], O = N[3], Z = N[8], G = N[9], H = N[10], V = N[11];
  if (N !== K)
    K[4] = N[4], K[5] = N[5], K[6] = N[6], K[7] = N[7], K[12] = N[12], K[13] = N[13], K[14] = N[14], K[15] = N[15];
  return K[0] = Q * $ - Z * Y, K[1] = X * $ - G * Y, K[2] = B * $ - H * Y, K[3] = O * $ - V * Y, K[8] = Q * Y + Z * $, K[9] = X * Y + G * $, K[10] = B * Y + H * $, K[11] = O * Y + V * $, K;
};
var IK = function(K, N, W) {
  var Y = Math.sin(W), $ = Math.cos(W), Q = N[0], X = N[1], B = N[2], O = N[3], Z = N[4], G = N[5], H = N[6], V = N[7];
  if (N !== K)
    K[8] = N[8], K[9] = N[9], K[10] = N[10], K[11] = N[11], K[12] = N[12], K[13] = N[13], K[14] = N[14], K[15] = N[15];
  return K[0] = Q * $ + Z * Y, K[1] = X * $ + G * Y, K[2] = B * $ + H * Y, K[3] = O * $ + V * Y, K[4] = Z * $ - Q * Y, K[5] = G * $ - X * Y, K[6] = H * $ - B * Y, K[7] = V * $ - O * Y, K;
};
var UK = function(K, N) {
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = N[0], K[13] = N[1], K[14] = N[2], K[15] = 1, K;
};
var hK = function(K, N) {
  return K[0] = N[0], K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = N[1], K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = N[2], K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var PK = function(K, N, W) {
  var Y = W[0], $ = W[1], Q = W[2], X = Math.hypot(Y, $, Q), B, O, Z;
  if (X < D)
    return null;
  return X = 1 / X, Y *= X, $ *= X, Q *= X, B = Math.sin(N), O = Math.cos(N), Z = 1 - O, K[0] = Y * Y * Z + O, K[1] = $ * Y * Z + Q * B, K[2] = Q * Y * Z - $ * B, K[3] = 0, K[4] = Y * $ * Z - Q * B, K[5] = $ * $ * Z + O, K[6] = Q * $ * Z + Y * B, K[7] = 0, K[8] = Y * Q * Z + $ * B, K[9] = $ * Q * Z - Y * B, K[10] = Q * Q * Z + O, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var AK = function(K, N) {
  var W = Math.sin(N), Y = Math.cos(N);
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = Y, K[6] = W, K[7] = 0, K[8] = 0, K[9] = -W, K[10] = Y, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var DK = function(K, N) {
  var W = Math.sin(N), Y = Math.cos(N);
  return K[0] = Y, K[1] = 0, K[2] = -W, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = W, K[9] = 0, K[10] = Y, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var RK = function(K, N) {
  var W = Math.sin(N), Y = Math.cos(N);
  return K[0] = Y, K[1] = W, K[2] = 0, K[3] = 0, K[4] = -W, K[5] = Y, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var J0 = function(K, N, W) {
  var Y = N[0], $ = N[1], Q = N[2], X = N[3], B = Y + Y, O = $ + $, Z = Q + Q, G = Y * B, H = Y * O, V = Y * Z, C = $ * O, L = $ * Z, E = Q * Z, J = X * B, I = X * O, R = X * Z;
  return K[0] = 1 - (C + E), K[1] = H + R, K[2] = V - I, K[3] = 0, K[4] = H - R, K[5] = 1 - (G + E), K[6] = L + J, K[7] = 0, K[8] = V + I, K[9] = L - J, K[10] = 1 - (G + C), K[11] = 0, K[12] = W[0], K[13] = W[1], K[14] = W[2], K[15] = 1, K;
};
var TK = function(K, N) {
  var W = new _(3), Y = -N[0], $ = -N[1], Q = -N[2], X = N[3], B = N[4], O = N[5], Z = N[6], G = N[7], H = Y * Y + $ * $ + Q * Q + X * X;
  if (H > 0)
    W[0] = (B * X + G * Y + O * Q - Z * $) * 2 / H, W[1] = (O * X + G * $ + Z * Y - B * Q) * 2 / H, W[2] = (Z * X + G * Q + B * $ - O * Y) * 2 / H;
  else
    W[0] = (B * X + G * Y + O * Q - Z * $) * 2, W[1] = (O * X + G * $ + Z * Y - B * Q) * 2, W[2] = (Z * X + G * Q + B * $ - O * Y) * 2;
  return J0(K, N, W), K;
};
var SK = function(K, N) {
  return K[0] = N[12], K[1] = N[13], K[2] = N[14], K;
};
var H0 = function(K, N) {
  var W = N[0], Y = N[1], $ = N[2], Q = N[4], X = N[5], B = N[6], O = N[8], Z = N[9], G = N[10];
  return K[0] = Math.hypot(W, Y, $), K[1] = Math.hypot(Q, X, B), K[2] = Math.hypot(O, Z, G), K;
};
var _K = function(K, N) {
  var W = new _(3);
  H0(W, N);
  var Y = 1 / W[0], $ = 1 / W[1], Q = 1 / W[2], X = N[0] * Y, B = N[1] * $, O = N[2] * Q, Z = N[4] * Y, G = N[5] * $, H = N[6] * Q, V = N[8] * Y, C = N[9] * $, L = N[10] * Q, E = X + G + L, J = 0;
  if (E > 0)
    J = Math.sqrt(E + 1) * 2, K[3] = 0.25 * J, K[0] = (H - C) / J, K[1] = (V - O) / J, K[2] = (B - Z) / J;
  else if (X > G && X > L)
    J = Math.sqrt(1 + X - G - L) * 2, K[3] = (H - C) / J, K[0] = 0.25 * J, K[1] = (B + Z) / J, K[2] = (V + O) / J;
  else if (G > L)
    J = Math.sqrt(1 + G - X - L) * 2, K[3] = (V - O) / J, K[0] = (B + Z) / J, K[1] = 0.25 * J, K[2] = (H + C) / J;
  else
    J = Math.sqrt(1 + L - X - G) * 2, K[3] = (B - Z) / J, K[0] = (V + O) / J, K[1] = (H + C) / J, K[2] = 0.25 * J;
  return K;
};
var kK = function(K, N, W, Y) {
  var $ = N[0], Q = N[1], X = N[2], B = N[3], O = $ + $, Z = Q + Q, G = X + X, H = $ * O, V = $ * Z, C = $ * G, L = Q * Z, E = Q * G, J = X * G, I = B * O, R = B * Z, A = B * G, P = Y[0], U = Y[1], h = Y[2];
  return K[0] = (1 - (L + J)) * P, K[1] = (V + A) * P, K[2] = (C - R) * P, K[3] = 0, K[4] = (V - A) * U, K[5] = (1 - (H + J)) * U, K[6] = (E + I) * U, K[7] = 0, K[8] = (C + R) * h, K[9] = (E - I) * h, K[10] = (1 - (H + L)) * h, K[11] = 0, K[12] = W[0], K[13] = W[1], K[14] = W[2], K[15] = 1, K;
};
var jK = function(K, N, W, Y, $) {
  var Q = N[0], X = N[1], B = N[2], O = N[3], Z = Q + Q, G = X + X, H = B + B, V = Q * Z, C = Q * G, L = Q * H, E = X * G, J = X * H, I = B * H, R = O * Z, A = O * G, P = O * H, U = Y[0], h = Y[1], g = Y[2], k = $[0], j = $[1], M = $[2], p = (1 - (E + I)) * U, F = (C + P) * U, q = (L - A) * U, T = (C - P) * h, f = (1 - (V + I)) * h, l = (J + R) * h, c = (L + A) * g, X0 = (J - R) * g, B0 = (1 - (V + E)) * g;
  return K[0] = p, K[1] = F, K[2] = q, K[3] = 0, K[4] = T, K[5] = f, K[6] = l, K[7] = 0, K[8] = c, K[9] = X0, K[10] = B0, K[11] = 0, K[12] = W[0] + k - (p * k + T * j + c * M), K[13] = W[1] + j - (F * k + f * j + X0 * M), K[14] = W[2] + M - (q * k + l * j + B0 * M), K[15] = 1, K;
};
var MK = function(K, N) {
  var W = N[0], Y = N[1], $ = N[2], Q = N[3], X = W + W, B = Y + Y, O = $ + $, Z = W * X, G = Y * X, H = Y * B, V = $ * X, C = $ * B, L = $ * O, E = Q * X, J = Q * B, I = Q * O;
  return K[0] = 1 - H - L, K[1] = G + I, K[2] = V - J, K[3] = 0, K[4] = G - I, K[5] = 1 - Z - L, K[6] = C + E, K[7] = 0, K[8] = V + J, K[9] = C - E, K[10] = 1 - Z - H, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var gK = function(K, N, W, Y, $, Q, X) {
  var B = 1 / (W - N), O = 1 / ($ - Y), Z = 1 / (Q - X);
  return K[0] = Q * 2 * B, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = Q * 2 * O, K[6] = 0, K[7] = 0, K[8] = (W + N) * B, K[9] = ($ + Y) * O, K[10] = (X + Q) * Z, K[11] = -1, K[12] = 0, K[13] = 0, K[14] = X * Q * 2 * Z, K[15] = 0, K;
};
var C0 = function(K, N, W, Y, $) {
  var Q = 1 / Math.tan(N / 2), X;
  if (K[0] = Q / W, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = Q, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = -1, K[12] = 0, K[13] = 0, K[15] = 0, $ != null && $ !== Infinity)
    X = 1 / (Y - $), K[10] = ($ + Y) * X, K[14] = 2 * $ * Y * X;
  else
    K[10] = -1, K[14] = -2 * Y;
  return K;
};
var FK = function(K, N, W, Y, $) {
  var Q = 1 / Math.tan(N / 2), X;
  if (K[0] = Q / W, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = Q, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = -1, K[12] = 0, K[13] = 0, K[15] = 0, $ != null && $ !== Infinity)
    X = 1 / (Y - $), K[10] = $ * X, K[14] = $ * Y * X;
  else
    K[10] = -1, K[14] = -Y;
  return K;
};
var qK = function(K, N, W, Y) {
  var $ = Math.tan(N.upDegrees * Math.PI / 180), Q = Math.tan(N.downDegrees * Math.PI / 180), X = Math.tan(N.leftDegrees * Math.PI / 180), B = Math.tan(N.rightDegrees * Math.PI / 180), O = 2 / (X + B), Z = 2 / ($ + Q);
  return K[0] = O, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = Z, K[6] = 0, K[7] = 0, K[8] = -((X - B) * O * 0.5), K[9] = ($ - Q) * Z * 0.5, K[10] = Y / (W - Y), K[11] = -1, K[12] = 0, K[13] = 0, K[14] = Y * W / (W - Y), K[15] = 0, K;
};
var V0 = function(K, N, W, Y, $, Q, X) {
  var B = 1 / (N - W), O = 1 / (Y - $), Z = 1 / (Q - X);
  return K[0] = -2 * B, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = -2 * O, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 2 * Z, K[11] = 0, K[12] = (N + W) * B, K[13] = ($ + Y) * O, K[14] = (X + Q) * Z, K[15] = 1, K;
};
var wK = function(K, N, W, Y, $, Q, X) {
  var B = 1 / (N - W), O = 1 / (Y - $), Z = 1 / (Q - X);
  return K[0] = -2 * B, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = -2 * O, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = Z, K[11] = 0, K[12] = (N + W) * B, K[13] = ($ + Y) * O, K[14] = Q * Z, K[15] = 1, K;
};
var dK = function(K, N, W, Y) {
  var $, Q, X, B, O, Z, G, H, V, C, L = N[0], E = N[1], J = N[2], I = Y[0], R = Y[1], A = Y[2], P = W[0], U = W[1], h = W[2];
  if (Math.abs(L - P) < D && Math.abs(E - U) < D && Math.abs(J - h) < D)
    return Z0(K);
  if (G = L - P, H = E - U, V = J - h, C = 1 / Math.hypot(G, H, V), G *= C, H *= C, V *= C, $ = R * V - A * H, Q = A * G - I * V, X = I * H - R * G, C = Math.hypot($, Q, X), !C)
    $ = 0, Q = 0, X = 0;
  else
    C = 1 / C, $ *= C, Q *= C, X *= C;
  if (B = H * X - V * Q, O = V * $ - G * X, Z = G * Q - H * $, C = Math.hypot(B, O, Z), !C)
    B = 0, O = 0, Z = 0;
  else
    C = 1 / C, B *= C, O *= C, Z *= C;
  return K[0] = $, K[1] = B, K[2] = G, K[3] = 0, K[4] = Q, K[5] = O, K[6] = H, K[7] = 0, K[8] = X, K[9] = Z, K[10] = V, K[11] = 0, K[12] = -($ * L + Q * E + X * J), K[13] = -(B * L + O * E + Z * J), K[14] = -(G * L + H * E + V * J), K[15] = 1, K;
};
var nK = function(K, N, W, Y) {
  var $ = N[0], Q = N[1], X = N[2], B = Y[0], O = Y[1], Z = Y[2], G = $ - W[0], H = Q - W[1], V = X - W[2], C = G * G + H * H + V * V;
  if (C > 0)
    C = 1 / Math.sqrt(C), G *= C, H *= C, V *= C;
  var L = O * V - Z * H, E = Z * G - B * V, J = B * H - O * G;
  if (C = L * L + E * E + J * J, C > 0)
    C = 1 / Math.sqrt(C), L *= C, E *= C, J *= C;
  return K[0] = L, K[1] = E, K[2] = J, K[3] = 0, K[4] = H * J - V * E, K[5] = V * L - G * J, K[6] = G * E - H * L, K[7] = 0, K[8] = G, K[9] = H, K[10] = V, K[11] = 0, K[12] = $, K[13] = Q, K[14] = X, K[15] = 1, K;
};
var vK = function(K) {
  return "mat4(" + K[0] + ", " + K[1] + ", " + K[2] + ", " + K[3] + ", " + K[4] + ", " + K[5] + ", " + K[6] + ", " + K[7] + ", " + K[8] + ", " + K[9] + ", " + K[10] + ", " + K[11] + ", " + K[12] + ", " + K[13] + ", " + K[14] + ", " + K[15] + ")";
};
var iK = function(K) {
  return Math.hypot(K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8], K[9], K[10], K[11], K[12], K[13], K[14], K[15]);
};
var lK = function(K, N, W) {
  return K[0] = N[0] + W[0], K[1] = N[1] + W[1], K[2] = N[2] + W[2], K[3] = N[3] + W[3], K[4] = N[4] + W[4], K[5] = N[5] + W[5], K[6] = N[6] + W[6], K[7] = N[7] + W[7], K[8] = N[8] + W[8], K[9] = N[9] + W[9], K[10] = N[10] + W[10], K[11] = N[11] + W[11], K[12] = N[12] + W[12], K[13] = N[13] + W[13], K[14] = N[14] + W[14], K[15] = N[15] + W[15], K;
};
var E0 = function(K, N, W) {
  return K[0] = N[0] - W[0], K[1] = N[1] - W[1], K[2] = N[2] - W[2], K[3] = N[3] - W[3], K[4] = N[4] - W[4], K[5] = N[5] - W[5], K[6] = N[6] - W[6], K[7] = N[7] - W[7], K[8] = N[8] - W[8], K[9] = N[9] - W[9], K[10] = N[10] - W[10], K[11] = N[11] - W[11], K[12] = N[12] - W[12], K[13] = N[13] - W[13], K[14] = N[14] - W[14], K[15] = N[15] - W[15], K;
};
var cK = function(K, N, W) {
  return K[0] = N[0] * W, K[1] = N[1] * W, K[2] = N[2] * W, K[3] = N[3] * W, K[4] = N[4] * W, K[5] = N[5] * W, K[6] = N[6] * W, K[7] = N[7] * W, K[8] = N[8] * W, K[9] = N[9] * W, K[10] = N[10] * W, K[11] = N[11] * W, K[12] = N[12] * W, K[13] = N[13] * W, K[14] = N[14] * W, K[15] = N[15] * W, K;
};
var zK = function(K, N, W, Y) {
  return K[0] = N[0] + W[0] * Y, K[1] = N[1] + W[1] * Y, K[2] = N[2] + W[2] * Y, K[3] = N[3] + W[3] * Y, K[4] = N[4] + W[4] * Y, K[5] = N[5] + W[5] * Y, K[6] = N[6] + W[6] * Y, K[7] = N[7] + W[7] * Y, K[8] = N[8] + W[8] * Y, K[9] = N[9] + W[9] * Y, K[10] = N[10] + W[10] * Y, K[11] = N[11] + W[11] * Y, K[12] = N[12] + W[12] * Y, K[13] = N[13] + W[13] * Y, K[14] = N[14] + W[14] * Y, K[15] = N[15] + W[15] * Y, K;
};
var yK = function(K, N) {
  return K[0] === N[0] && K[1] === N[1] && K[2] === N[2] && K[3] === N[3] && K[4] === N[4] && K[5] === N[5] && K[6] === N[6] && K[7] === N[7] && K[8] === N[8] && K[9] === N[9] && K[10] === N[10] && K[11] === N[11] && K[12] === N[12] && K[13] === N[13] && K[14] === N[14] && K[15] === N[15];
};
var rK = function(K, N) {
  var W = K[0], Y = K[1], $ = K[2], Q = K[3], X = K[4], B = K[5], O = K[6], Z = K[7], G = K[8], H = K[9], V = K[10], C = K[11], L = K[12], E = K[13], J = K[14], I = K[15], R = N[0], A = N[1], P = N[2], U = N[3], h = N[4], g = N[5], k = N[6], j = N[7], M = N[8], p = N[9], F = N[10], q = N[11], T = N[12], f = N[13], l = N[14], c = N[15];
  return Math.abs(W - R) <= D * Math.max(1, Math.abs(W), Math.abs(R)) && Math.abs(Y - A) <= D * Math.max(1, Math.abs(Y), Math.abs(A)) && Math.abs($ - P) <= D * Math.max(1, Math.abs($), Math.abs(P)) && Math.abs(Q - U) <= D * Math.max(1, Math.abs(Q), Math.abs(U)) && Math.abs(X - h) <= D * Math.max(1, Math.abs(X), Math.abs(h)) && Math.abs(B - g) <= D * Math.max(1, Math.abs(B), Math.abs(g)) && Math.abs(O - k) <= D * Math.max(1, Math.abs(O), Math.abs(k)) && Math.abs(Z - j) <= D * Math.max(1, Math.abs(Z), Math.abs(j)) && Math.abs(G - M) <= D * Math.max(1, Math.abs(G), Math.abs(M)) && Math.abs(H - p) <= D * Math.max(1, Math.abs(H), Math.abs(p)) && Math.abs(V - F) <= D * Math.max(1, Math.abs(V), Math.abs(F)) && Math.abs(C - q) <= D * Math.max(1, Math.abs(C), Math.abs(q)) && Math.abs(L - T) <= D * Math.max(1, Math.abs(L), Math.abs(T)) && Math.abs(E - f) <= D * Math.max(1, Math.abs(E), Math.abs(f)) && Math.abs(J - l) <= D * Math.max(1, Math.abs(J), Math.abs(l)) && Math.abs(I - c) <= D * Math.max(1, Math.abs(I), Math.abs(c));
};
var x = function() {
  var K = new _(3);
  if (_ != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0;
  return K;
};
var xK = function(K) {
  var N = new _(3);
  return N[0] = K[0], N[1] = K[1], N[2] = K[2], N;
};
var L0 = function(K) {
  var N = K[0], W = K[1], Y = K[2];
  return Math.hypot(N, W, Y);
};
var e = function(K, N, W) {
  var Y = new _(3);
  return Y[0] = K, Y[1] = N, Y[2] = W, Y;
};
var eK = function(K, N) {
  return K[0] = N[0], K[1] = N[1], K[2] = N[2], K;
};
var bK = function(K, N, W, Y) {
  return K[0] = N, K[1] = W, K[2] = Y, K;
};
var uK = function(K, N, W) {
  return K[0] = N[0] + W[0], K[1] = N[1] + W[1], K[2] = N[2] + W[2], K;
};
var I0 = function(K, N, W) {
  return K[0] = N[0] - W[0], K[1] = N[1] - W[1], K[2] = N[2] - W[2], K;
};
var U0 = function(K, N, W) {
  return K[0] = N[0] * W[0], K[1] = N[1] * W[1], K[2] = N[2] * W[2], K;
};
var h0 = function(K, N, W) {
  return K[0] = N[0] / W[0], K[1] = N[1] / W[1], K[2] = N[2] / W[2], K;
};
var oK = function(K, N) {
  return K[0] = Math.ceil(N[0]), K[1] = Math.ceil(N[1]), K[2] = Math.ceil(N[2]), K;
};
var tK = function(K, N) {
  return K[0] = Math.floor(N[0]), K[1] = Math.floor(N[1]), K[2] = Math.floor(N[2]), K;
};
var aK = function(K, N, W) {
  return K[0] = Math.min(N[0], W[0]), K[1] = Math.min(N[1], W[1]), K[2] = Math.min(N[2], W[2]), K;
};
var KN = function(K, N, W) {
  return K[0] = Math.max(N[0], W[0]), K[1] = Math.max(N[1], W[1]), K[2] = Math.max(N[2], W[2]), K;
};
var NN = function(K, N) {
  return K[0] = Math.round(N[0]), K[1] = Math.round(N[1]), K[2] = Math.round(N[2]), K;
};
var WN = function(K, N, W) {
  return K[0] = N[0] * W, K[1] = N[1] * W, K[2] = N[2] * W, K;
};
var YN = function(K, N, W, Y) {
  return K[0] = N[0] + W[0] * Y, K[1] = N[1] + W[1] * Y, K[2] = N[2] + W[2] * Y, K;
};
var P0 = function(K, N) {
  var W = N[0] - K[0], Y = N[1] - K[1], $ = N[2] - K[2];
  return Math.hypot(W, Y, $);
};
var A0 = function(K, N) {
  var W = N[0] - K[0], Y = N[1] - K[1], $ = N[2] - K[2];
  return W * W + Y * Y + $ * $;
};
var D0 = function(K) {
  var N = K[0], W = K[1], Y = K[2];
  return N * N + W * W + Y * Y;
};
var $N = function(K, N) {
  return K[0] = -N[0], K[1] = -N[1], K[2] = -N[2], K;
};
var QN = function(K, N) {
  return K[0] = 1 / N[0], K[1] = 1 / N[1], K[2] = 1 / N[2], K;
};
var a = function(K, N) {
  var W = N[0], Y = N[1], $ = N[2], Q = W * W + Y * Y + $ * $;
  if (Q > 0)
    Q = 1 / Math.sqrt(Q);
  return K[0] = N[0] * Q, K[1] = N[1] * Q, K[2] = N[2] * Q, K;
};
var b = function(K, N) {
  return K[0] * N[0] + K[1] * N[1] + K[2] * N[2];
};
var y = function(K, N, W) {
  var Y = N[0], $ = N[1], Q = N[2], X = W[0], B = W[1], O = W[2];
  return K[0] = $ * O - Q * B, K[1] = Q * X - Y * O, K[2] = Y * B - $ * X, K;
};
var XN = function(K, N, W, Y) {
  var $ = N[0], Q = N[1], X = N[2];
  return K[0] = $ + Y * (W[0] - $), K[1] = Q + Y * (W[1] - Q), K[2] = X + Y * (W[2] - X), K;
};
var BN = function(K, N, W, Y, $, Q) {
  var X = Q * Q, B = X * (2 * Q - 3) + 1, O = X * (Q - 2) + Q, Z = X * (Q - 1), G = X * (3 - 2 * Q);
  return K[0] = N[0] * B + W[0] * O + Y[0] * Z + $[0] * G, K[1] = N[1] * B + W[1] * O + Y[1] * Z + $[1] * G, K[2] = N[2] * B + W[2] * O + Y[2] * Z + $[2] * G, K;
};
var ON = function(K, N, W, Y, $, Q) {
  var X = 1 - Q, B = X * X, O = Q * Q, Z = B * X, G = 3 * Q * B, H = 3 * O * X, V = O * Q;
  return K[0] = N[0] * Z + W[0] * G + Y[0] * H + $[0] * V, K[1] = N[1] * Z + W[1] * G + Y[1] * H + $[1] * V, K[2] = N[2] * Z + W[2] * G + Y[2] * H + $[2] * V, K;
};
var ZN = function(K, N) {
  N = N || 1;
  var W = n() * 2 * Math.PI, Y = n() * 2 - 1, $ = Math.sqrt(1 - Y * Y) * N;
  return K[0] = Math.cos(W) * $, K[1] = Math.sin(W) * $, K[2] = Y * N, K;
};
var GN = function(K, N, W) {
  var Y = N[0], $ = N[1], Q = N[2], X = W[3] * Y + W[7] * $ + W[11] * Q + W[15];
  return X = X || 1, K[0] = (W[0] * Y + W[4] * $ + W[8] * Q + W[12]) / X, K[1] = (W[1] * Y + W[5] * $ + W[9] * Q + W[13]) / X, K[2] = (W[2] * Y + W[6] * $ + W[10] * Q + W[14]) / X, K;
};
var JN = function(K, N, W) {
  var Y = N[0], $ = N[1], Q = N[2];
  return K[0] = Y * W[0] + $ * W[3] + Q * W[6], K[1] = Y * W[1] + $ * W[4] + Q * W[7], K[2] = Y * W[2] + $ * W[5] + Q * W[8], K;
};
var HN = function(K, N, W) {
  var Y = W[0], $ = W[1], Q = W[2], X = W[3], B = N[0], O = N[1], Z = N[2], G = $ * Z - Q * O, H = Q * B - Y * Z, V = Y * O - $ * B, C = $ * V - Q * H, L = Q * G - Y * V, E = Y * H - $ * G, J = X * 2;
  return G *= J, H *= J, V *= J, C *= 2, L *= 2, E *= 2, K[0] = B + G + C, K[1] = O + H + L, K[2] = Z + V + E, K;
};
var CN = function(K, N, W, Y) {
  var $ = [], Q = [];
  return $[0] = N[0] - W[0], $[1] = N[1] - W[1], $[2] = N[2] - W[2], Q[0] = $[0], Q[1] = $[1] * Math.cos(Y) - $[2] * Math.sin(Y), Q[2] = $[1] * Math.sin(Y) + $[2] * Math.cos(Y), K[0] = Q[0] + W[0], K[1] = Q[1] + W[1], K[2] = Q[2] + W[2], K;
};
var VN = function(K, N, W, Y) {
  var $ = [], Q = [];
  return $[0] = N[0] - W[0], $[1] = N[1] - W[1], $[2] = N[2] - W[2], Q[0] = $[2] * Math.sin(Y) + $[0] * Math.cos(Y), Q[1] = $[1], Q[2] = $[2] * Math.cos(Y) - $[0] * Math.sin(Y), K[0] = Q[0] + W[0], K[1] = Q[1] + W[1], K[2] = Q[2] + W[2], K;
};
var EN = function(K, N, W, Y) {
  var $ = [], Q = [];
  return $[0] = N[0] - W[0], $[1] = N[1] - W[1], $[2] = N[2] - W[2], Q[0] = $[0] * Math.cos(Y) - $[1] * Math.sin(Y), Q[1] = $[0] * Math.sin(Y) + $[1] * Math.cos(Y), Q[2] = $[2], K[0] = Q[0] + W[0], K[1] = Q[1] + W[1], K[2] = Q[2] + W[2], K;
};
var LN = function(K, N) {
  var W = K[0], Y = K[1], $ = K[2], Q = N[0], X = N[1], B = N[2], O = Math.sqrt(W * W + Y * Y + $ * $), Z = Math.sqrt(Q * Q + X * X + B * B), G = O * Z, H = G && b(K, N) / G;
  return Math.acos(Math.min(Math.max(H, -1), 1));
};
var IN = function(K) {
  return K[0] = 0, K[1] = 0, K[2] = 0, K;
};
var UN = function(K) {
  return "vec3(" + K[0] + ", " + K[1] + ", " + K[2] + ")";
};
var hN = function(K, N) {
  return K[0] === N[0] && K[1] === N[1] && K[2] === N[2];
};
var PN = function(K, N) {
  var W = K[0], Y = K[1], $ = K[2], Q = N[0], X = N[1], B = N[2];
  return Math.abs(W - Q) <= D * Math.max(1, Math.abs(W), Math.abs(Q)) && Math.abs(Y - X) <= D * Math.max(1, Math.abs(Y), Math.abs(X)) && Math.abs($ - B) <= D * Math.max(1, Math.abs($), Math.abs(B));
};
var jN = function() {
  var K = new _(4);
  if (_ != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0, K[3] = 0;
  return K;
};
var R0 = function(K) {
  var N = new _(4);
  return N[0] = K[0], N[1] = K[1], N[2] = K[2], N[3] = K[3], N;
};
var T0 = function(K, N, W, Y) {
  var $ = new _(4);
  return $[0] = K, $[1] = N, $[2] = W, $[3] = Y, $;
};
var S0 = function(K, N) {
  return K[0] = N[0], K[1] = N[1], K[2] = N[2], K[3] = N[3], K;
};
var _0 = function(K, N, W, Y, $) {
  return K[0] = N, K[1] = W, K[2] = Y, K[3] = $, K;
};
var k0 = function(K, N, W) {
  return K[0] = N[0] + W[0], K[1] = N[1] + W[1], K[2] = N[2] + W[2], K[3] = N[3] + W[3], K;
};
var j0 = function(K, N, W) {
  return K[0] = N[0] * W, K[1] = N[1] * W, K[2] = N[2] * W, K[3] = N[3] * W, K;
};
var M0 = function(K) {
  var N = K[0], W = K[1], Y = K[2], $ = K[3];
  return Math.hypot(N, W, Y, $);
};
var g0 = function(K) {
  var N = K[0], W = K[1], Y = K[2], $ = K[3];
  return N * N + W * W + Y * Y + $ * $;
};
var p0 = function(K, N) {
  var W = N[0], Y = N[1], $ = N[2], Q = N[3], X = W * W + Y * Y + $ * $ + Q * Q;
  if (X > 0)
    X = 1 / Math.sqrt(X);
  return K[0] = W * X, K[1] = Y * X, K[2] = $ * X, K[3] = Q * X, K;
};
var F0 = function(K, N) {
  return K[0] * N[0] + K[1] * N[1] + K[2] * N[2] + K[3] * N[3];
};
var q0 = function(K, N, W, Y) {
  var $ = N[0], Q = N[1], X = N[2], B = N[3];
  return K[0] = $ + Y * (W[0] - $), K[1] = Q + Y * (W[1] - Q), K[2] = X + Y * (W[2] - X), K[3] = B + Y * (W[3] - B), K;
};
var f0 = function(K, N) {
  return K[0] === N[0] && K[1] === N[1] && K[2] === N[2] && K[3] === N[3];
};
var w0 = function(K, N) {
  var W = K[0], Y = K[1], $ = K[2], Q = K[3], X = N[0], B = N[1], O = N[2], Z = N[3];
  return Math.abs(W - X) <= D * Math.max(1, Math.abs(W), Math.abs(X)) && Math.abs(Y - B) <= D * Math.max(1, Math.abs(Y), Math.abs(B)) && Math.abs($ - O) <= D * Math.max(1, Math.abs($), Math.abs(O)) && Math.abs(Q - Z) <= D * Math.max(1, Math.abs(Q), Math.abs(Z));
};
var N0 = function() {
  var K = new _(4);
  if (_ != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0;
  return K[3] = 1, K;
};
var gN = function(K) {
  return K[0] = 0, K[1] = 0, K[2] = 0, K[3] = 1, K;
};
var d0 = function(K, N, W) {
  W = W * 0.5;
  var Y = Math.sin(W);
  return K[0] = Y * N[0], K[1] = Y * N[1], K[2] = Y * N[2], K[3] = Math.cos(W), K;
};
var pN = function(K, N) {
  var W = Math.acos(N[3]) * 2, Y = Math.sin(W / 2);
  if (Y > D)
    K[0] = N[0] / Y, K[1] = N[1] / Y, K[2] = N[2] / Y;
  else
    K[0] = 1, K[1] = 0, K[2] = 0;
  return W;
};
var FN = function(K, N) {
  var W = z0(K, N);
  return Math.acos(2 * W * W - 1);
};
var n0 = function(K, N, W) {
  var Y = N[0], $ = N[1], Q = N[2], X = N[3], B = W[0], O = W[1], Z = W[2], G = W[3];
  return K[0] = Y * G + X * B + $ * Z - Q * O, K[1] = $ * G + X * O + Q * B - Y * Z, K[2] = Q * G + X * Z + Y * O - $ * B, K[3] = X * G - Y * B - $ * O - Q * Z, K;
};
var qN = function(K, N, W) {
  W *= 0.5;
  var Y = N[0], $ = N[1], Q = N[2], X = N[3], B = Math.sin(W), O = Math.cos(W);
  return K[0] = Y * O + X * B, K[1] = $ * O + Q * B, K[2] = Q * O - $ * B, K[3] = X * O - Y * B, K;
};
var fN = function(K, N, W) {
  W *= 0.5;
  var Y = N[0], $ = N[1], Q = N[2], X = N[3], B = Math.sin(W), O = Math.cos(W);
  return K[0] = Y * O - Q * B, K[1] = $ * O + X * B, K[2] = Q * O + Y * B, K[3] = X * O - $ * B, K;
};
var wN = function(K, N, W) {
  W *= 0.5;
  var Y = N[0], $ = N[1], Q = N[2], X = N[3], B = Math.sin(W), O = Math.cos(W);
  return K[0] = Y * O + $ * B, K[1] = $ * O - Y * B, K[2] = Q * O + X * B, K[3] = X * O - Q * B, K;
};
var dN = function(K, N) {
  var W = N[0], Y = N[1], $ = N[2];
  return K[0] = W, K[1] = Y, K[2] = $, K[3] = Math.sqrt(Math.abs(1 - W * W - Y * Y - $ * $)), K;
};
var v0 = function(K, N) {
  var W = N[0], Y = N[1], $ = N[2], Q = N[3], X = Math.sqrt(W * W + Y * Y + $ * $), B = Math.exp(Q), O = X > 0 ? B * Math.sin(X) / X : 0;
  return K[0] = W * O, K[1] = Y * O, K[2] = $ * O, K[3] = B * Math.cos(X), K;
};
var i0 = function(K, N) {
  var W = N[0], Y = N[1], $ = N[2], Q = N[3], X = Math.sqrt(W * W + Y * Y + $ * $), B = X > 0 ? Math.atan2(X, Q) / X : 0;
  return K[0] = W * B, K[1] = Y * B, K[2] = $ * B, K[3] = 0.5 * Math.log(W * W + Y * Y + $ * $ + Q * Q), K;
};
var nN = function(K, N, W) {
  return i0(K, N), c0(K, K, W), v0(K, K), K;
};
var u = function(K, N, W, Y) {
  var $ = N[0], Q = N[1], X = N[2], B = N[3], O = W[0], Z = W[1], G = W[2], H = W[3], V, C, L, E, J;
  if (C = $ * O + Q * Z + X * G + B * H, C < 0)
    C = -C, O = -O, Z = -Z, G = -G, H = -H;
  if (1 - C > D)
    V = Math.acos(C), L = Math.sin(V), E = Math.sin((1 - Y) * V) / L, J = Math.sin(Y * V) / L;
  else
    E = 1 - Y, J = Y;
  return K[0] = E * $ + J * O, K[1] = E * Q + J * Z, K[2] = E * X + J * G, K[3] = E * B + J * H, K;
};
var vN = function(K) {
  var N = n(), W = n(), Y = n(), $ = Math.sqrt(1 - N), Q = Math.sqrt(N);
  return K[0] = $ * Math.sin(2 * Math.PI * W), K[1] = $ * Math.cos(2 * Math.PI * W), K[2] = Q * Math.sin(2 * Math.PI * Y), K[3] = Q * Math.cos(2 * Math.PI * Y), K;
};
var iN = function(K, N) {
  var W = N[0], Y = N[1], $ = N[2], Q = N[3], X = W * W + Y * Y + $ * $ + Q * Q, B = X ? 1 / X : 0;
  return K[0] = -W * B, K[1] = -Y * B, K[2] = -$ * B, K[3] = Q * B, K;
};
var lN = function(K, N) {
  return K[0] = -N[0], K[1] = -N[1], K[2] = -N[2], K[3] = N[3], K;
};
var l0 = function(K, N) {
  var W = N[0] + N[4] + N[8], Y;
  if (W > 0)
    Y = Math.sqrt(W + 1), K[3] = 0.5 * Y, Y = 0.5 / Y, K[0] = (N[5] - N[7]) * Y, K[1] = (N[6] - N[2]) * Y, K[2] = (N[1] - N[3]) * Y;
  else {
    var $ = 0;
    if (N[4] > N[0])
      $ = 1;
    if (N[8] > N[$ * 3 + $])
      $ = 2;
    var Q = ($ + 1) % 3, X = ($ + 2) % 3;
    Y = Math.sqrt(N[$ * 3 + $] - N[Q * 3 + Q] - N[X * 3 + X] + 1), K[$] = 0.5 * Y, Y = 0.5 / Y, K[3] = (N[Q * 3 + X] - N[X * 3 + Q]) * Y, K[Q] = (N[Q * 3 + $] + N[$ * 3 + Q]) * Y, K[X] = (N[X * 3 + $] + N[$ * 3 + X]) * Y;
  }
  return K;
};
var cN = function(K, N, W, Y) {
  var $ = 0.5 * Math.PI / 180;
  N *= $, W *= $, Y *= $;
  var Q = Math.sin(N), X = Math.cos(N), B = Math.sin(W), O = Math.cos(W), Z = Math.sin(Y), G = Math.cos(Y);
  return K[0] = Q * O * G - X * B * Z, K[1] = X * B * G + Q * O * Z, K[2] = X * O * Z - Q * B * G, K[3] = X * O * G + Q * B * Z, K;
};
var zN = function(K) {
  return "quat(" + K[0] + ", " + K[1] + ", " + K[2] + ", " + K[3] + ")";
};
var KK = Object.defineProperty;
var t = (K, N) => {
  for (var W in N)
    KK(K, W, { get: N[W], enumerable: true, configurable: true, set: (Y) => N[W] = () => Y });
};
var D = 0.000001;
var _ = typeof Float32Array !== "undefined" ? Float32Array : Array;
var n = Math.random;
var JW = Math.PI / 180;
if (!Math.hypot)
  Math.hypot = function() {
    var K = 0, N = arguments.length;
    while (N--)
      K += arguments[N] * arguments[N];
    return Math.sqrt(K);
  };
var S = {};
t(S, { transpose: () => {
  {
    return OK;
  }
}, translate: () => {
  {
    return HK;
  }
}, targetTo: () => {
  {
    return nK;
  }
}, subtract: () => {
  {
    return E0;
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
    return CK;
  }
}, rotateZ: () => {
  {
    return IK;
  }
}, rotateY: () => {
  {
    return LK;
  }
}, rotateX: () => {
  {
    return EK;
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
    return C0;
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
    return zK;
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
    return dK;
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
    return SK;
  }
}, getScaling: () => {
  {
    return H0;
  }
}, getRotation: () => {
  {
    return _K;
  }
}, frustum: () => {
  {
    return gK;
  }
}, fromZRotation: () => {
  {
    return RK;
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
    return XK;
  }
}, fromTranslation: () => {
  {
    return UK;
  }
}, fromScaling: () => {
  {
    return hK;
  }
}, fromRotationTranslationScaleOrigin: () => {
  {
    return jK;
  }
}, fromRotationTranslationScale: () => {
  {
    return kK;
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
    return TK;
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
    return yK;
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
    return YK;
  }
}, copy: () => {
  {
    return QK;
  }
}, clone: () => {
  {
    return $K;
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
var pK = C0;
var fK = V0;
var sK = G0;
var mK = E0;
var s = {};
t(s, { str: () => {
  {
    return zN;
  }
}, squaredLength: () => {
  {
    return r0;
  }
}, sqrLen: () => {
  {
    return oN;
  }
}, sqlerp: () => {
  {
    return NW;
  }
}, slerp: () => {
  {
    return u;
  }
}, setAxisAngle: () => {
  {
    return d0;
  }
}, setAxes: () => {
  {
    return WW;
  }
}, set: () => {
  {
    return mN;
  }
}, scale: () => {
  {
    return c0;
  }
}, rotationTo: () => {
  {
    return KW;
  }
}, rotateZ: () => {
  {
    return wN;
  }
}, rotateY: () => {
  {
    return fN;
  }
}, rotateX: () => {
  {
    return qN;
  }
}, random: () => {
  {
    return vN;
  }
}, pow: () => {
  {
    return nN;
  }
}, normalize: () => {
  {
    return W0;
  }
}, multiply: () => {
  {
    return n0;
  }
}, mul: () => {
  {
    return eN;
  }
}, ln: () => {
  {
    return i0;
  }
}, lerp: () => {
  {
    return bN;
  }
}, length: () => {
  {
    return y0;
  }
}, len: () => {
  {
    return uN;
  }
}, invert: () => {
  {
    return iN;
  }
}, identity: () => {
  {
    return gN;
  }
}, getAxisAngle: () => {
  {
    return pN;
  }
}, getAngle: () => {
  {
    return FN;
  }
}, fromValues: () => {
  {
    return rN;
  }
}, fromMat3: () => {
  {
    return l0;
  }
}, fromEuler: () => {
  {
    return cN;
  }
}, exp: () => {
  {
    return v0;
  }
}, exactEquals: () => {
  {
    return tN;
  }
}, equals: () => {
  {
    return aN;
  }
}, dot: () => {
  {
    return z0;
  }
}, create: () => {
  {
    return N0;
  }
}, copy: () => {
  {
    return sN;
  }
}, conjugate: () => {
  {
    return lN;
  }
}, clone: () => {
  {
    return yN;
  }
}, calculateW: () => {
  {
    return dN;
  }
}, add: () => {
  {
    return xN;
  }
} });
var r = {};
t(r, { zero: () => {
  {
    return IN;
  }
}, transformQuat: () => {
  {
    return HN;
  }
}, transformMat4: () => {
  {
    return GN;
  }
}, transformMat3: () => {
  {
    return JN;
  }
}, subtract: () => {
  {
    return I0;
  }
}, sub: () => {
  {
    return AN;
  }
}, str: () => {
  {
    return UN;
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
    return _N;
  }
}, sqrDist: () => {
  {
    return SN;
  }
}, set: () => {
  {
    return bK;
  }
}, scaleAndAdd: () => {
  {
    return YN;
  }
}, scale: () => {
  {
    return WN;
  }
}, round: () => {
  {
    return NN;
  }
}, rotateZ: () => {
  {
    return EN;
  }
}, rotateY: () => {
  {
    return VN;
  }
}, rotateX: () => {
  {
    return CN;
  }
}, random: () => {
  {
    return ZN;
  }
}, normalize: () => {
  {
    return a;
  }
}, negate: () => {
  {
    return $N;
  }
}, multiply: () => {
  {
    return U0;
  }
}, mul: () => {
  {
    return DN;
  }
}, min: () => {
  {
    return aK;
  }
}, max: () => {
  {
    return KN;
  }
}, lerp: () => {
  {
    return XN;
  }
}, length: () => {
  {
    return L0;
  }
}, len: () => {
  {
    return K0;
  }
}, inverse: () => {
  {
    return QN;
  }
}, hermite: () => {
  {
    return BN;
  }
}, fromValues: () => {
  {
    return e;
  }
}, forEach: () => {
  {
    return kN;
  }
}, floor: () => {
  {
    return tK;
  }
}, exactEquals: () => {
  {
    return hN;
  }
}, equals: () => {
  {
    return PN;
  }
}, dot: () => {
  {
    return b;
  }
}, divide: () => {
  {
    return h0;
  }
}, div: () => {
  {
    return RN;
  }
}, distance: () => {
  {
    return P0;
  }
}, dist: () => {
  {
    return TN;
  }
}, cross: () => {
  {
    return y;
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
    return ON;
  }
}, angle: () => {
  {
    return LN;
  }
}, add: () => {
  {
    return uK;
  }
} });
var AN = I0;
var DN = U0;
var RN = h0;
var TN = P0;
var SN = A0;
var K0 = L0;
var _N = D0;
var kN = function() {
  var K = x();
  return function(N, W, Y, $, Q, X) {
    var B, O;
    if (!W)
      W = 3;
    if (!Y)
      Y = 0;
    if ($)
      O = Math.min($ * W + Y, N.length);
    else
      O = N.length;
    for (B = Y;B < O; B += W)
      K[0] = N[B], K[1] = N[B + 1], K[2] = N[B + 2], Q(K, K, X), N[B] = K[0], N[B + 1] = K[1], N[B + 2] = K[2];
    return N;
  };
}();
var HW = function() {
  var K = jN();
  return function(N, W, Y, $, Q, X) {
    var B, O;
    if (!W)
      W = 4;
    if (!Y)
      Y = 0;
    if ($)
      O = Math.min($ * W + Y, N.length);
    else
      O = N.length;
    for (B = Y;B < O; B += W)
      K[0] = N[B], K[1] = N[B + 1], K[2] = N[B + 2], K[3] = N[B + 3], Q(K, K, X), N[B] = K[0], N[B + 1] = K[1], N[B + 2] = K[2], N[B + 3] = K[3];
    return N;
  };
}();
var yN = R0;
var rN = T0;
var sN = S0;
var mN = _0;
var xN = k0;
var eN = n0;
var c0 = j0;
var z0 = F0;
var bN = q0;
var y0 = M0;
var uN = y0;
var r0 = g0;
var oN = r0;
var W0 = p0;
var tN = f0;
var aN = w0;
var KW = function() {
  var K = x(), N = e(1, 0, 0), W = e(0, 1, 0);
  return function(Y, $, Q) {
    var X = b($, Q);
    if (X < -0.999999) {
      if (y(K, N, $), K0(K) < 0.000001)
        y(K, W, $);
      return a(K, K), d0(Y, K, Math.PI), Y;
    } else if (X > 0.999999)
      return Y[0] = 0, Y[1] = 0, Y[2] = 0, Y[3] = 1, Y;
    else
      return y(K, $, Q), Y[0] = K[0], Y[1] = K[1], Y[2] = K[2], Y[3] = 1 + X, W0(Y, Y);
  };
}();
var NW = function() {
  var K = N0(), N = N0();
  return function(W, Y, $, Q, X, B) {
    return u(K, Y, X, B), u(N, $, Q, B), u(W, K, N, 2 * B * (1 - B)), W;
  };
}();
var WW = function() {
  var K = O0();
  return function(N, W, Y, $) {
    return K[0] = Y[0], K[3] = Y[1], K[6] = Y[2], K[1] = $[0], K[4] = $[1], K[7] = $[2], K[2] = -W[0], K[5] = -W[1], K[8] = -W[2], W0(N, l0(N, K));
  };
}();
var YW = Math.PI / 90;
var Y0 = [0, 0, 0];
var s0 = S.create();
var m0 = S.create();
var o = s.create();

class m {
  #K = Float32Array.from(S.create());
  static HIDDEN = m.create().scale(0, 0, 0);
  static IDENTITY = m.create();
  constructor() {
    this.identity();
  }
  static create() {
    return new m;
  }
  copy(K) {
    return S.copy(this.#K, K.getMatrix()), this;
  }
  identity() {
    return S.identity(this.#K), this;
  }
  invert(K) {
    return S.invert(this.#K, K?.getMatrix() ?? this.getMatrix()), this;
  }
  multiply(K) {
    return S.multiply(this.#K, this.#K, K.getMatrix()), this;
  }
  multiply2(K, N) {
    return S.multiply(this.#K, K.getMatrix(), N.getMatrix()), this;
  }
  multiply3(K, N, W) {
    return this.multiply2(K, N), this.multiply(W), this;
  }
  translate(K, N, W) {
    const Y = Y0;
    return Y[0] = K, Y[1] = N, Y[2] = W, this.move(Y);
  }
  move(K) {
    return S.translate(this.#K, this.#K, K), this;
  }
  rotateX(K) {
    return S.rotateX(this.#K, this.#K, K), this;
  }
  rotateY(K) {
    return S.rotateY(this.#K, this.#K, K), this;
  }
  rotateZ(K) {
    return S.rotateZ(this.#K, this.#K, K), this;
  }
  setXRotation(K) {
    return S.fromXRotation(this.getMatrix(), K), this;
  }
  setYRotation(K) {
    return S.fromYRotation(this.getMatrix(), K), this;
  }
  scale(K, N, W) {
    return S.scale(this.#K, this.#K, [K, N ?? K, W ?? K]), this;
  }
  perspective(K, N, W, Y) {
    return S.perspective(this.#K, K * YW, N, W, Y), this;
  }
  ortho(K, N, W, Y, $, Q) {
    return S.ortho(this.#K, K, N, W, Y, $, Q), this;
  }
  combine(K, N, W = 0.5) {
    return S.multiplyScalar(s0, K.getMatrix(), 1 - W), S.multiplyScalar(m0, N.getMatrix(), W), S.add(this.#K, s0, m0), this;
  }
  static getMoveVector(K, N, W, Y) {
    const $ = Y0;
    if ($[0] = K, $[1] = N, $[2] = W, Y)
      S.getRotation(o, Y.getMatrix()), s.invert(o, o), r.transformQuat($, $, o);
    return $;
  }
  getPosition() {
    const K = Y0;
    return K[0] = this.#K[12], K[1] = this.#K[13], K[2] = this.#K[14], K;
  }
  setVector(K) {
    return this.setPosition(K[0], K[1], K[2]);
  }
  setPosition(K, N, W) {
    return this.#K[12] = K, this.#K[13] = N, this.#K[14] = W, this;
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
  #N = false;
  #Y = 0;
  #W;
  #$;
  constructor(K, N, W) {
    this.getValue = N, this.apply = W, this.#$ = K, this.#K = this.getValue(K);
  }
  set element(K) {
    this.#$ = K, this.#K = this.getValue(K), this.#W = undefined;
  }
  setGoal(K, N, W) {
    if (this.#W && this.#W !== W)
      return;
    if (this.#K !== K || this.#Y !== N)
      this.#Y = N, this.#K = K, this.#W = W, this.#N = true;
  }
  get goal() {
    return this.#K;
  }
  update(K) {
    if (this.#N) {
      const N = this.getValue(this.#$), W = this.goal - N, Y = Math.min(Math.abs(W), this.#Y * K);
      if (Y <= 0.01)
        this.apply(this.#$, this.goal), this.#N = false, this.#W = undefined;
      else
        this.apply(this.#$, N + Y * Math.sign(W));
    }
    return this.#N;
  }
}

class e0 {
  i;
  f;
  warningLimit = 50000;
  #K = new Set;
  #N = [];
  constructor(K, N) {
    this.initCall = K, this.onRecycle = N;
  }
  create(...K) {
    const N = this.#N.pop();
    if (N)
      return this.#K.add(N), this.initCall(N, ...K);
    const W = this.initCall(undefined, ...K);
    return this.#K.add(W), this.#W(), W;
  }
  recycle(K) {
    this.#K.delete(K), this.#Y(K);
  }
  recycleAll() {
    for (let K of this.#K)
      this.#Y(K);
    this.#K.clear();
  }
  clear() {
    this.#N.length = 0, this.#K.clear();
  }
  countObjectsInExistence() {
    return this.#K.size + this.#N.length;
  }
  #Y(K) {
    this.#N.push(K), this.onRecycle?.(K);
  }
  #W() {
    if (this.countObjectsInExistence() === this.warningLimit)
      console.warn("ObjectPool already created", this.#K.size + this.#N.length, "in", this.constructor.name);
  }
}

class b0 extends e0 {
  constructor() {
    super((K, N) => {
      if (!K)
        return new x0(N, (W) => W.valueOf(), (W, Y) => W.setValue(Y));
      return K.element = N, K;
    });
  }
}
var $W = new b0;

class v {
  w;
  q;
  #K = 0;
  #N;
  constructor(K = 0, N, W = $W) {
    this.onChange = N, this.pool = W, this.#K = K;
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
    if (this.#N) {
      const N = !!this.#N.update(K);
      if (!N)
        this.pool.recycle(this.#N), this.#N = undefined;
      return N;
    }
    return false;
  }
  refresh({ deltaTime: K, stopUpdate: N }) {
    if (!this.update(K))
      N();
  }
  progressTowards(K, N, W, Y) {
    if (!this.#N)
      this.#N = this.pool.create(this);
    if (this.#N.setGoal(K, N, W), Y)
      Y.loop(this, undefined);
  }
  get goal() {
    return this.#N?.goal ?? this.valueOf();
  }
}
class o0 {
  i;
  f;
  warningLimit = 50000;
  #K = new Set;
  #N = [];
  constructor(K, N) {
    this.initCall = K, this.onRecycle = N;
  }
  create(...K) {
    const N = this.#N.pop();
    if (N)
      return this.#K.add(N), this.initCall(N, ...K);
    const W = this.initCall(undefined, ...K);
    return this.#K.add(W), this.#W(), W;
  }
  recycle(K) {
    this.#K.delete(K), this.#Y(K);
  }
  recycleAll() {
    for (let K of this.#K)
      this.#Y(K);
    this.#K.clear();
  }
  clear() {
    this.#N.length = 0, this.#K.clear();
  }
  countObjectsInExistence() {
    return this.#K.size + this.#N.length;
  }
  #Y(K) {
    this.#N.push(K), this.onRecycle?.(K);
  }
  #W() {
    if (this.countObjectsInExistence() === this.warningLimit)
      console.warn("ObjectPool already created", this.#K.size + this.#N.length, "in", this.constructor.name);
  }
}

class $0 extends o0 {
  constructor() {
    super((K, N, W, Y) => {
      if (!K)
        return [N, W, Y];
      return K[0] = N, K[1] = W, K[2] = Y, K;
    });
  }
}

class i {
  vectorPool;
  motor;
  data;
  constructor({ motor: K, vectorPool: N }) {
    this.motor = K, this.vectorPool = N ?? new $0, this.data = { vectorPool: this.vectorPool };
  }
  refresh({ data: { vectorPool: K } }) {
    K.recycleAll();
  }
  transformToPosition(K) {
    this.motor.scheduleUpdate(this, this.data);
    const N = K.getMatrix();
    return this.vectorPool.create(N[12], N[13], N[14]);
  }
  toVector(K, N, W) {
    return this.motor.scheduleUpdate(this, this.data), this.vectorPool.create(K, N, W);
  }
  static toVector(K, N, W, Y) {
    return Y[0] = K, Y[1] = N, Y[2] = W, Y;
  }
  static transformToPosition(K, N) {
    const W = K.getMatrix();
    N[0] = W[12], N[1] = W[13], N[2] = W[14];
  }
}
var d;
(function(Y) {
  Y[Y["AT_POSITION"] = 0] = "AT_POSITION";
  Y[Y["MOVED"] = 1] = "MOVED";
  Y[Y["BLOCKED"] = 2] = "BLOCKED";
})(d || (d = {}));
var Q0 = function(K, N) {
  if (K) {
    const W = K.length.valueOf();
    for (let Y = 0;Y < W; Y++) {
      const $ = K.at(Y);
      if ($ !== undefined && N($, Y))
        return true;
    }
  }
  return false;
};

class t0 {
  #K = w.create().setPosition(0, 0, 0);
  #N = new Set;
  #Y = [0, 0, 0];
  position = [0, 0, 0];
  #W;
  constructor({ blockers: K }, N) {
    if (this.#W = K, N)
      this.onChange(N);
  }
  onChange(K) {
    return this.#N.add(K), this;
  }
  removeChangeListener(K) {
    this.#N.delete(K);
  }
  changedPosition(K, N, W) {
    i.transformToPosition(this.#K, this.position);
    for (let Y of this.#N)
      Y(K, N, W);
  }
  moveBy(K, N, W, Y) {
    const $ = w.getMoveVector(K, N, W, Y), Q = Q0(this.#W, (X) => X.isBlocked(i.toVector(this.position[0] + $[0], this.position[1] + $[1], this.position[2] + $[2], this.#Y), this.position));
    if (!Q)
      if ($[0] || $[1] || $[2])
        this.#K.move($), this.changedPosition(K, N, W);
      else
        return d.AT_POSITION;
    return Q ? d.BLOCKED : d.MOVED;
  }
  moveTo(K, N, W) {
    if (this.position[0] === K && this.position[1] === N && this.position[2] === W)
      return d.AT_POSITION;
    const Y = Q0(this.#W, ($) => $.isBlocked(i.toVector(K, N, W, this.#Y), this.position));
    if (!Y) {
      const [$, Q, X] = this.#K.getPosition();
      if ($ !== K || Q !== N || X !== W) {
        const B = K - $, O = N - Q, Z = W - X;
        this.#K.setPosition(K, N, W), this.changedPosition(B, O, Z);
      }
    }
    return Y ? d.BLOCKED : d.MOVED;
  }
  movedTo(K, N, W) {
    return this.moveTo(K, N, W), this;
  }
  gotoPos(K, N, W, Y = 0.1) {
    const $ = this.position, Q = K - $[0], X = N - $[1], B = W - $[2], O = Math.sqrt(Q * Q + X * X + B * B);
    if (O > 0.01) {
      const Z = Math.min(O, Y);
      return this.moveBy(Q / O * Z, X / O * Z, B / O * Z);
    } else
      return this.moveTo(K, N, W);
  }
  getMatrix() {
    return this.#K.getMatrix();
  }
}
var BW = 1;
var OW = 1;

class a0 {
  K;
  #K = w.create();
  #N = w.create();
  #Y = w.create();
  #W = [0, 0];
  perspective;
  zoom;
  constructor(K) {
    this.onChange = K;
    this.perspective = new v(BW, K), this.zoom = new v(OW, (N) => {
      this.configure(this.#W, N);
    });
  }
  configPerspectiveMatrix(K, N, W, Y) {
    this.#N.perspective(K, N, W, Y);
  }
  configOrthoMatrix(K, N, W, Y) {
    this.#Y.ortho(-K / 2, K / 2, -N / 2, N / 2, W, Y);
  }
  configure(K, N, W = 0.5, Y = 1e4) {
    if (!N)
      N = this.zoom.valueOf();
    this.#W[0] = K[0], this.#W[1] = K[1];
    const $ = this.#W[0] / this.#W[1], Q = 45 / Math.sqrt(N);
    this.configPerspectiveMatrix(Q, $, Math.max(W, 0.00001), Y), this.configOrthoMatrix($ / N / N, 1 / N / N, -Y, Y), this.onChange?.();
  }
  getMatrix() {
    return this.#K.combine(this.#Y, this.#N, this.perspective.valueOf()), this.#K.getMatrix();
  }
}
export {
  w as Matrix
};
