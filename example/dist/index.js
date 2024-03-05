// /Users/vincent/dok-matrix/example/node_modules/dok-matrix/dist/index.js
var J0 = function() {
  var B = new R(9);
  if (R != Float32Array)
    B[1] = 0, B[2] = 0, B[3] = 0, B[5] = 0, B[6] = 0, B[7] = 0;
  return B[0] = 1, B[4] = 1, B[8] = 1, B;
};
var XB = function() {
  var B = new R(16);
  if (R != Float32Array)
    B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0;
  return B[0] = 1, B[5] = 1, B[10] = 1, B[15] = 1, B;
};
var YB = function(B) {
  var K = new R(16);
  return K[0] = B[0], K[1] = B[1], K[2] = B[2], K[3] = B[3], K[4] = B[4], K[5] = B[5], K[6] = B[6], K[7] = B[7], K[8] = B[8], K[9] = B[9], K[10] = B[10], K[11] = B[11], K[12] = B[12], K[13] = B[13], K[14] = B[14], K[15] = B[15], K;
};
var $B = function(B, K) {
  return B[0] = K[0], B[1] = K[1], B[2] = K[2], B[3] = K[3], B[4] = K[4], B[5] = K[5], B[6] = K[6], B[7] = K[7], B[8] = K[8], B[9] = K[9], B[10] = K[10], B[11] = K[11], B[12] = K[12], B[13] = K[13], B[14] = K[14], B[15] = K[15], B;
};
var QB = function(B, K, W, X, Y, $, Q, Z, H, J, O, L, C, V, E, U) {
  var G = new R(16);
  return G[0] = B, G[1] = K, G[2] = W, G[3] = X, G[4] = Y, G[5] = $, G[6] = Q, G[7] = Z, G[8] = H, G[9] = J, G[10] = O, G[11] = L, G[12] = C, G[13] = V, G[14] = E, G[15] = U, G;
};
var ZB = function(B, K, W, X, Y, $, Q, Z, H, J, O, L, C, V, E, U, G) {
  return B[0] = K, B[1] = W, B[2] = X, B[3] = Y, B[4] = $, B[5] = Q, B[6] = Z, B[7] = H, B[8] = J, B[9] = O, B[10] = L, B[11] = C, B[12] = V, B[13] = E, B[14] = U, B[15] = G, B;
};
var O0 = function(B) {
  return B[0] = 1, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = 1, B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[10] = 1, B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0, B[15] = 1, B;
};
var HB = function(B, K) {
  if (B === K) {
    var W = K[1], X = K[2], Y = K[3], $ = K[6], Q = K[7], Z = K[11];
    B[1] = K[4], B[2] = K[8], B[3] = K[12], B[4] = W, B[6] = K[9], B[7] = K[13], B[8] = X, B[9] = $, B[11] = K[14], B[12] = Y, B[13] = Q, B[14] = Z;
  } else
    B[0] = K[0], B[1] = K[4], B[2] = K[8], B[3] = K[12], B[4] = K[1], B[5] = K[5], B[6] = K[9], B[7] = K[13], B[8] = K[2], B[9] = K[6], B[10] = K[10], B[11] = K[14], B[12] = K[3], B[13] = K[7], B[14] = K[11], B[15] = K[15];
  return B;
};
var JB = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3], Q = K[4], Z = K[5], H = K[6], J = K[7], O = K[8], L = K[9], C = K[10], V = K[11], E = K[12], U = K[13], G = K[14], I = K[15], S = W * Z - X * Q, k = W * H - Y * Q, A = W * J - $ * Q, D = X * H - Y * Z, P = X * J - $ * Z, F = Y * J - $ * H, j = O * U - L * E, h = O * G - C * E, M = O * I - V * E, p = L * G - C * U, g = L * I - V * U, q = C * I - V * G, T = S * q - k * g + A * p + D * M - P * h + F * j;
  if (!T)
    return null;
  return T = 1 / T, B[0] = (Z * q - H * g + J * p) * T, B[1] = (Y * g - X * q - $ * p) * T, B[2] = (U * F - G * P + I * D) * T, B[3] = (C * P - L * F - V * D) * T, B[4] = (H * M - Q * q - J * h) * T, B[5] = (W * q - Y * M + $ * h) * T, B[6] = (G * A - E * F - I * k) * T, B[7] = (O * F - C * A + V * k) * T, B[8] = (Q * g - Z * M + J * j) * T, B[9] = (X * M - W * g - $ * j) * T, B[10] = (E * P - U * A + I * S) * T, B[11] = (L * A - O * P - V * S) * T, B[12] = (Z * h - Q * p - H * j) * T, B[13] = (W * p - X * h + Y * j) * T, B[14] = (U * k - E * D - G * S) * T, B[15] = (O * D - L * k + C * S) * T, B;
};
var OB = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3], Q = K[4], Z = K[5], H = K[6], J = K[7], O = K[8], L = K[9], C = K[10], V = K[11], E = K[12], U = K[13], G = K[14], I = K[15];
  return B[0] = Z * (C * I - V * G) - L * (H * I - J * G) + U * (H * V - J * C), B[1] = -(X * (C * I - V * G) - L * (Y * I - $ * G) + U * (Y * V - $ * C)), B[2] = X * (H * I - J * G) - Z * (Y * I - $ * G) + U * (Y * J - $ * H), B[3] = -(X * (H * V - J * C) - Z * (Y * V - $ * C) + L * (Y * J - $ * H)), B[4] = -(Q * (C * I - V * G) - O * (H * I - J * G) + E * (H * V - J * C)), B[5] = W * (C * I - V * G) - O * (Y * I - $ * G) + E * (Y * V - $ * C), B[6] = -(W * (H * I - J * G) - Q * (Y * I - $ * G) + E * (Y * J - $ * H)), B[7] = W * (H * V - J * C) - Q * (Y * V - $ * C) + O * (Y * J - $ * H), B[8] = Q * (L * I - V * U) - O * (Z * I - J * U) + E * (Z * V - J * L), B[9] = -(W * (L * I - V * U) - O * (X * I - $ * U) + E * (X * V - $ * L)), B[10] = W * (Z * I - J * U) - Q * (X * I - $ * U) + E * (X * J - $ * Z), B[11] = -(W * (Z * V - J * L) - Q * (X * V - $ * L) + O * (X * J - $ * Z)), B[12] = -(Q * (L * G - C * U) - O * (Z * G - H * U) + E * (Z * C - H * L)), B[13] = W * (L * G - C * U) - O * (X * G - Y * U) + E * (X * C - Y * L), B[14] = -(W * (Z * G - H * U) - Q * (X * G - Y * U) + E * (X * H - Y * Z)), B[15] = W * (Z * C - H * L) - Q * (X * C - Y * L) + O * (X * H - Y * Z), B;
};
var GB = function(B) {
  var K = B[0], W = B[1], X = B[2], Y = B[3], $ = B[4], Q = B[5], Z = B[6], H = B[7], J = B[8], O = B[9], L = B[10], C = B[11], V = B[12], E = B[13], U = B[14], G = B[15], I = K * Q - W * $, S = K * Z - X * $, k = K * H - Y * $, A = W * Z - X * Q, D = W * H - Y * Q, P = X * H - Y * Z, F = J * E - O * V, j = J * U - L * V, h = J * G - C * V, M = O * U - L * E, p = O * G - C * E, g = L * G - C * U;
  return I * g - S * p + k * M + A * h - D * j + P * F;
};
var G0 = function(B, K, W) {
  var X = K[0], Y = K[1], $ = K[2], Q = K[3], Z = K[4], H = K[5], J = K[6], O = K[7], L = K[8], C = K[9], V = K[10], E = K[11], U = K[12], G = K[13], I = K[14], S = K[15], k = W[0], A = W[1], D = W[2], P = W[3];
  return B[0] = k * X + A * Z + D * L + P * U, B[1] = k * Y + A * H + D * C + P * G, B[2] = k * $ + A * J + D * V + P * I, B[3] = k * Q + A * O + D * E + P * S, k = W[4], A = W[5], D = W[6], P = W[7], B[4] = k * X + A * Z + D * L + P * U, B[5] = k * Y + A * H + D * C + P * G, B[6] = k * $ + A * J + D * V + P * I, B[7] = k * Q + A * O + D * E + P * S, k = W[8], A = W[9], D = W[10], P = W[11], B[8] = k * X + A * Z + D * L + P * U, B[9] = k * Y + A * H + D * C + P * G, B[10] = k * $ + A * J + D * V + P * I, B[11] = k * Q + A * O + D * E + P * S, k = W[12], A = W[13], D = W[14], P = W[15], B[12] = k * X + A * Z + D * L + P * U, B[13] = k * Y + A * H + D * C + P * G, B[14] = k * $ + A * J + D * V + P * I, B[15] = k * Q + A * O + D * E + P * S, B;
};
var LB = function(B, K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q, Z, H, J, O, L, C, V, E, U, G, I;
  if (K === B)
    B[12] = K[0] * X + K[4] * Y + K[8] * $ + K[12], B[13] = K[1] * X + K[5] * Y + K[9] * $ + K[13], B[14] = K[2] * X + K[6] * Y + K[10] * $ + K[14], B[15] = K[3] * X + K[7] * Y + K[11] * $ + K[15];
  else
    Q = K[0], Z = K[1], H = K[2], J = K[3], O = K[4], L = K[5], C = K[6], V = K[7], E = K[8], U = K[9], G = K[10], I = K[11], B[0] = Q, B[1] = Z, B[2] = H, B[3] = J, B[4] = O, B[5] = L, B[6] = C, B[7] = V, B[8] = E, B[9] = U, B[10] = G, B[11] = I, B[12] = Q * X + O * Y + E * $ + K[12], B[13] = Z * X + L * Y + U * $ + K[13], B[14] = H * X + C * Y + G * $ + K[14], B[15] = J * X + V * Y + I * $ + K[15];
  return B;
};
var VB = function(B, K, W) {
  var X = W[0], Y = W[1], $ = W[2];
  return B[0] = K[0] * X, B[1] = K[1] * X, B[2] = K[2] * X, B[3] = K[3] * X, B[4] = K[4] * Y, B[5] = K[5] * Y, B[6] = K[6] * Y, B[7] = K[7] * Y, B[8] = K[8] * $, B[9] = K[9] * $, B[10] = K[10] * $, B[11] = K[11] * $, B[12] = K[12], B[13] = K[13], B[14] = K[14], B[15] = K[15], B;
};
var CB = function(B, K, W, X) {
  var Y = X[0], $ = X[1], Q = X[2], Z = Math.hypot(Y, $, Q), H, J, O, L, C, V, E, U, G, I, S, k, A, D, P, F, j, h, M, p, g, q, T, f;
  if (Z < N)
    return null;
  if (Z = 1 / Z, Y *= Z, $ *= Z, Q *= Z, H = Math.sin(W), J = Math.cos(W), O = 1 - J, L = K[0], C = K[1], V = K[2], E = K[3], U = K[4], G = K[5], I = K[6], S = K[7], k = K[8], A = K[9], D = K[10], P = K[11], F = Y * Y * O + J, j = $ * Y * O + Q * H, h = Q * Y * O - $ * H, M = Y * $ * O - Q * H, p = $ * $ * O + J, g = Q * $ * O + Y * H, q = Y * Q * O + $ * H, T = $ * Q * O - Y * H, f = Q * Q * O + J, B[0] = L * F + U * j + k * h, B[1] = C * F + G * j + A * h, B[2] = V * F + I * j + D * h, B[3] = E * F + S * j + P * h, B[4] = L * M + U * p + k * g, B[5] = C * M + G * p + A * g, B[6] = V * M + I * p + D * g, B[7] = E * M + S * p + P * g, B[8] = L * q + U * T + k * f, B[9] = C * q + G * T + A * f, B[10] = V * q + I * T + D * f, B[11] = E * q + S * T + P * f, K !== B)
    B[12] = K[12], B[13] = K[13], B[14] = K[14], B[15] = K[15];
  return B;
};
var UB = function(B, K, W) {
  var X = Math.sin(W), Y = Math.cos(W), $ = K[4], Q = K[5], Z = K[6], H = K[7], J = K[8], O = K[9], L = K[10], C = K[11];
  if (K !== B)
    B[0] = K[0], B[1] = K[1], B[2] = K[2], B[3] = K[3], B[12] = K[12], B[13] = K[13], B[14] = K[14], B[15] = K[15];
  return B[4] = $ * Y + J * X, B[5] = Q * Y + O * X, B[6] = Z * Y + L * X, B[7] = H * Y + C * X, B[8] = J * Y - $ * X, B[9] = O * Y - Q * X, B[10] = L * Y - Z * X, B[11] = C * Y - H * X, B;
};
var EB = function(B, K, W) {
  var X = Math.sin(W), Y = Math.cos(W), $ = K[0], Q = K[1], Z = K[2], H = K[3], J = K[8], O = K[9], L = K[10], C = K[11];
  if (K !== B)
    B[4] = K[4], B[5] = K[5], B[6] = K[6], B[7] = K[7], B[12] = K[12], B[13] = K[13], B[14] = K[14], B[15] = K[15];
  return B[0] = $ * Y - J * X, B[1] = Q * Y - O * X, B[2] = Z * Y - L * X, B[3] = H * Y - C * X, B[8] = $ * X + J * Y, B[9] = Q * X + O * Y, B[10] = Z * X + L * Y, B[11] = H * X + C * Y, B;
};
var IB = function(B, K, W) {
  var X = Math.sin(W), Y = Math.cos(W), $ = K[0], Q = K[1], Z = K[2], H = K[3], J = K[4], O = K[5], L = K[6], C = K[7];
  if (K !== B)
    B[8] = K[8], B[9] = K[9], B[10] = K[10], B[11] = K[11], B[12] = K[12], B[13] = K[13], B[14] = K[14], B[15] = K[15];
  return B[0] = $ * Y + J * X, B[1] = Q * Y + O * X, B[2] = Z * Y + L * X, B[3] = H * Y + C * X, B[4] = J * Y - $ * X, B[5] = O * Y - Q * X, B[6] = L * Y - Z * X, B[7] = C * Y - H * X, B;
};
var DB = function(B, K) {
  return B[0] = 1, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = 1, B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[10] = 1, B[11] = 0, B[12] = K[0], B[13] = K[1], B[14] = K[2], B[15] = 1, B;
};
var PB = function(B, K) {
  return B[0] = K[0], B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = K[1], B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[10] = K[2], B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0, B[15] = 1, B;
};
var AB = function(B, K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q = Math.hypot(X, Y, $), Z, H, J;
  if (Q < N)
    return null;
  return Q = 1 / Q, X *= Q, Y *= Q, $ *= Q, Z = Math.sin(K), H = Math.cos(K), J = 1 - H, B[0] = X * X * J + H, B[1] = Y * X * J + $ * Z, B[2] = $ * X * J - Y * Z, B[3] = 0, B[4] = X * Y * J - $ * Z, B[5] = Y * Y * J + H, B[6] = $ * Y * J + X * Z, B[7] = 0, B[8] = X * $ * J + Y * Z, B[9] = Y * $ * J - X * Z, B[10] = $ * $ * J + H, B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0, B[15] = 1, B;
};
var kB = function(B, K) {
  var W = Math.sin(K), X = Math.cos(K);
  return B[0] = 1, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = X, B[6] = W, B[7] = 0, B[8] = 0, B[9] = -W, B[10] = X, B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0, B[15] = 1, B;
};
var NB = function(B, K) {
  var W = Math.sin(K), X = Math.cos(K);
  return B[0] = X, B[1] = 0, B[2] = -W, B[3] = 0, B[4] = 0, B[5] = 1, B[6] = 0, B[7] = 0, B[8] = W, B[9] = 0, B[10] = X, B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0, B[15] = 1, B;
};
var SB = function(B, K) {
  var W = Math.sin(K), X = Math.cos(K);
  return B[0] = X, B[1] = W, B[2] = 0, B[3] = 0, B[4] = -W, B[5] = X, B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[10] = 1, B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0, B[15] = 1, B;
};
var L0 = function(B, K, W) {
  var X = K[0], Y = K[1], $ = K[2], Q = K[3], Z = X + X, H = Y + Y, J = $ + $, O = X * Z, L = X * H, C = X * J, V = Y * H, E = Y * J, U = $ * J, G = Q * Z, I = Q * H, S = Q * J;
  return B[0] = 1 - (V + U), B[1] = L + S, B[2] = C - I, B[3] = 0, B[4] = L - S, B[5] = 1 - (O + U), B[6] = E + G, B[7] = 0, B[8] = C + I, B[9] = E - G, B[10] = 1 - (O + V), B[11] = 0, B[12] = W[0], B[13] = W[1], B[14] = W[2], B[15] = 1, B;
};
var TB = function(B, K) {
  var W = new R(3), X = -K[0], Y = -K[1], $ = -K[2], Q = K[3], Z = K[4], H = K[5], J = K[6], O = K[7], L = X * X + Y * Y + $ * $ + Q * Q;
  if (L > 0)
    W[0] = (Z * Q + O * X + H * $ - J * Y) * 2 / L, W[1] = (H * Q + O * Y + J * X - Z * $) * 2 / L, W[2] = (J * Q + O * $ + Z * Y - H * X) * 2 / L;
  else
    W[0] = (Z * Q + O * X + H * $ - J * Y) * 2, W[1] = (H * Q + O * Y + J * X - Z * $) * 2, W[2] = (J * Q + O * $ + Z * Y - H * X) * 2;
  return L0(B, K, W), B;
};
var _B = function(B, K) {
  return B[0] = K[12], B[1] = K[13], B[2] = K[14], B;
};
var V0 = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[4], Q = K[5], Z = K[6], H = K[8], J = K[9], O = K[10];
  return B[0] = Math.hypot(W, X, Y), B[1] = Math.hypot($, Q, Z), B[2] = Math.hypot(H, J, O), B;
};
var RB = function(B, K) {
  var W = new R(3);
  V0(W, K);
  var X = 1 / W[0], Y = 1 / W[1], $ = 1 / W[2], Q = K[0] * X, Z = K[1] * Y, H = K[2] * $, J = K[4] * X, O = K[5] * Y, L = K[6] * $, C = K[8] * X, V = K[9] * Y, E = K[10] * $, U = Q + O + E, G = 0;
  if (U > 0)
    G = Math.sqrt(U + 1) * 2, B[3] = 0.25 * G, B[0] = (L - V) / G, B[1] = (C - H) / G, B[2] = (Z - J) / G;
  else if (Q > O && Q > E)
    G = Math.sqrt(1 + Q - O - E) * 2, B[3] = (L - V) / G, B[0] = 0.25 * G, B[1] = (Z + J) / G, B[2] = (C + H) / G;
  else if (O > E)
    G = Math.sqrt(1 + O - Q - E) * 2, B[3] = (C - H) / G, B[0] = (Z + J) / G, B[1] = 0.25 * G, B[2] = (L + V) / G;
  else
    G = Math.sqrt(1 + E - Q - O) * 2, B[3] = (Z - J) / G, B[0] = (C + H) / G, B[1] = (L + V) / G, B[2] = 0.25 * G;
  return B;
};
var jB = function(B, K, W, X) {
  var Y = K[0], $ = K[1], Q = K[2], Z = K[3], H = Y + Y, J = $ + $, O = Q + Q, L = Y * H, C = Y * J, V = Y * O, E = $ * J, U = $ * O, G = Q * O, I = Z * H, S = Z * J, k = Z * O, A = X[0], D = X[1], P = X[2];
  return B[0] = (1 - (E + G)) * A, B[1] = (C + k) * A, B[2] = (V - S) * A, B[3] = 0, B[4] = (C - k) * D, B[5] = (1 - (L + G)) * D, B[6] = (U + I) * D, B[7] = 0, B[8] = (V + S) * P, B[9] = (U - I) * P, B[10] = (1 - (L + E)) * P, B[11] = 0, B[12] = W[0], B[13] = W[1], B[14] = W[2], B[15] = 1, B;
};
var hB = function(B, K, W, X, Y) {
  var $ = K[0], Q = K[1], Z = K[2], H = K[3], J = $ + $, O = Q + Q, L = Z + Z, C = $ * J, V = $ * O, E = $ * L, U = Q * O, G = Q * L, I = Z * L, S = H * J, k = H * O, A = H * L, D = X[0], P = X[1], F = X[2], j = Y[0], h = Y[1], M = Y[2], p = (1 - (U + I)) * D, g = (V + A) * D, q = (E - k) * D, T = (V - A) * P, f = (1 - (C + I)) * P, c = (G + S) * P, i = (E + k) * F, Z0 = (G - S) * F, H0 = (1 - (C + U)) * F;
  return B[0] = p, B[1] = g, B[2] = q, B[3] = 0, B[4] = T, B[5] = f, B[6] = c, B[7] = 0, B[8] = i, B[9] = Z0, B[10] = H0, B[11] = 0, B[12] = W[0] + j - (p * j + T * h + i * M), B[13] = W[1] + h - (g * j + f * h + Z0 * M), B[14] = W[2] + M - (q * j + c * h + H0 * M), B[15] = 1, B;
};
var MB = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3], Q = W + W, Z = X + X, H = Y + Y, J = W * Q, O = X * Q, L = X * Z, C = Y * Q, V = Y * Z, E = Y * H, U = $ * Q, G = $ * Z, I = $ * H;
  return B[0] = 1 - L - E, B[1] = O + I, B[2] = C - G, B[3] = 0, B[4] = O - I, B[5] = 1 - J - E, B[6] = V + U, B[7] = 0, B[8] = C + G, B[9] = V - U, B[10] = 1 - J - L, B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0, B[15] = 1, B;
};
var FB = function(B, K, W, X, Y, $, Q) {
  var Z = 1 / (W - K), H = 1 / (Y - X), J = 1 / ($ - Q);
  return B[0] = $ * 2 * Z, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = $ * 2 * H, B[6] = 0, B[7] = 0, B[8] = (W + K) * Z, B[9] = (Y + X) * H, B[10] = (Q + $) * J, B[11] = -1, B[12] = 0, B[13] = 0, B[14] = Q * $ * 2 * J, B[15] = 0, B;
};
var C0 = function(B, K, W, X, Y) {
  var $ = 1 / Math.tan(K / 2), Q;
  if (B[0] = $ / W, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = $, B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[11] = -1, B[12] = 0, B[13] = 0, B[15] = 0, Y != null && Y !== Infinity)
    Q = 1 / (X - Y), B[10] = (Y + X) * Q, B[14] = 2 * Y * X * Q;
  else
    B[10] = -1, B[14] = -2 * X;
  return B;
};
var gB = function(B, K, W, X, Y) {
  var $ = 1 / Math.tan(K / 2), Q;
  if (B[0] = $ / W, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = $, B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[11] = -1, B[12] = 0, B[13] = 0, B[15] = 0, Y != null && Y !== Infinity)
    Q = 1 / (X - Y), B[10] = Y * Q, B[14] = Y * X * Q;
  else
    B[10] = -1, B[14] = -X;
  return B;
};
var qB = function(B, K, W, X) {
  var Y = Math.tan(K.upDegrees * Math.PI / 180), $ = Math.tan(K.downDegrees * Math.PI / 180), Q = Math.tan(K.leftDegrees * Math.PI / 180), Z = Math.tan(K.rightDegrees * Math.PI / 180), H = 2 / (Q + Z), J = 2 / (Y + $);
  return B[0] = H, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = J, B[6] = 0, B[7] = 0, B[8] = -((Q - Z) * H * 0.5), B[9] = (Y - $) * J * 0.5, B[10] = X / (W - X), B[11] = -1, B[12] = 0, B[13] = 0, B[14] = X * W / (W - X), B[15] = 0, B;
};
var U0 = function(B, K, W, X, Y, $, Q) {
  var Z = 1 / (K - W), H = 1 / (X - Y), J = 1 / ($ - Q);
  return B[0] = -2 * Z, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = -2 * H, B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[10] = 2 * J, B[11] = 0, B[12] = (K + W) * Z, B[13] = (Y + X) * H, B[14] = (Q + $) * J, B[15] = 1, B;
};
var fB = function(B, K, W, X, Y, $, Q) {
  var Z = 1 / (K - W), H = 1 / (X - Y), J = 1 / ($ - Q);
  return B[0] = -2 * Z, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = -2 * H, B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[10] = J, B[11] = 0, B[12] = (K + W) * Z, B[13] = (Y + X) * H, B[14] = $ * J, B[15] = 1, B;
};
var dB = function(B, K, W, X) {
  var Y, $, Q, Z, H, J, O, L, C, V, E = K[0], U = K[1], G = K[2], I = X[0], S = X[1], k = X[2], A = W[0], D = W[1], P = W[2];
  if (Math.abs(E - A) < N && Math.abs(U - D) < N && Math.abs(G - P) < N)
    return O0(B);
  if (O = E - A, L = U - D, C = G - P, V = 1 / Math.hypot(O, L, C), O *= V, L *= V, C *= V, Y = S * C - k * L, $ = k * O - I * C, Q = I * L - S * O, V = Math.hypot(Y, $, Q), !V)
    Y = 0, $ = 0, Q = 0;
  else
    V = 1 / V, Y *= V, $ *= V, Q *= V;
  if (Z = L * Q - C * $, H = C * Y - O * Q, J = O * $ - L * Y, V = Math.hypot(Z, H, J), !V)
    Z = 0, H = 0, J = 0;
  else
    V = 1 / V, Z *= V, H *= V, J *= V;
  return B[0] = Y, B[1] = Z, B[2] = O, B[3] = 0, B[4] = $, B[5] = H, B[6] = L, B[7] = 0, B[8] = Q, B[9] = J, B[10] = C, B[11] = 0, B[12] = -(Y * E + $ * U + Q * G), B[13] = -(Z * E + H * U + J * G), B[14] = -(O * E + L * U + C * G), B[15] = 1, B;
};
var vB = function(B, K, W, X) {
  var Y = K[0], $ = K[1], Q = K[2], Z = X[0], H = X[1], J = X[2], O = Y - W[0], L = $ - W[1], C = Q - W[2], V = O * O + L * L + C * C;
  if (V > 0)
    V = 1 / Math.sqrt(V), O *= V, L *= V, C *= V;
  var E = H * C - J * L, U = J * O - Z * C, G = Z * L - H * O;
  if (V = E * E + U * U + G * G, V > 0)
    V = 1 / Math.sqrt(V), E *= V, U *= V, G *= V;
  return B[0] = E, B[1] = U, B[2] = G, B[3] = 0, B[4] = L * G - C * U, B[5] = C * E - O * G, B[6] = O * U - L * E, B[7] = 0, B[8] = O, B[9] = L, B[10] = C, B[11] = 0, B[12] = Y, B[13] = $, B[14] = Q, B[15] = 1, B;
};
var nB = function(B) {
  return "mat4(" + B[0] + ", " + B[1] + ", " + B[2] + ", " + B[3] + ", " + B[4] + ", " + B[5] + ", " + B[6] + ", " + B[7] + ", " + B[8] + ", " + B[9] + ", " + B[10] + ", " + B[11] + ", " + B[12] + ", " + B[13] + ", " + B[14] + ", " + B[15] + ")";
};
var lB = function(B) {
  return Math.hypot(B[0], B[1], B[2], B[3], B[4], B[5], B[6], B[7], B[8], B[9], B[10], B[11], B[12], B[13], B[14], B[15]);
};
var cB = function(B, K, W) {
  return B[0] = K[0] + W[0], B[1] = K[1] + W[1], B[2] = K[2] + W[2], B[3] = K[3] + W[3], B[4] = K[4] + W[4], B[5] = K[5] + W[5], B[6] = K[6] + W[6], B[7] = K[7] + W[7], B[8] = K[8] + W[8], B[9] = K[9] + W[9], B[10] = K[10] + W[10], B[11] = K[11] + W[11], B[12] = K[12] + W[12], B[13] = K[13] + W[13], B[14] = K[14] + W[14], B[15] = K[15] + W[15], B;
};
var E0 = function(B, K, W) {
  return B[0] = K[0] - W[0], B[1] = K[1] - W[1], B[2] = K[2] - W[2], B[3] = K[3] - W[3], B[4] = K[4] - W[4], B[5] = K[5] - W[5], B[6] = K[6] - W[6], B[7] = K[7] - W[7], B[8] = K[8] - W[8], B[9] = K[9] - W[9], B[10] = K[10] - W[10], B[11] = K[11] - W[11], B[12] = K[12] - W[12], B[13] = K[13] - W[13], B[14] = K[14] - W[14], B[15] = K[15] - W[15], B;
};
var iB = function(B, K, W) {
  return B[0] = K[0] * W, B[1] = K[1] * W, B[2] = K[2] * W, B[3] = K[3] * W, B[4] = K[4] * W, B[5] = K[5] * W, B[6] = K[6] * W, B[7] = K[7] * W, B[8] = K[8] * W, B[9] = K[9] * W, B[10] = K[10] * W, B[11] = K[11] * W, B[12] = K[12] * W, B[13] = K[13] * W, B[14] = K[14] * W, B[15] = K[15] * W, B;
};
var yB = function(B, K, W, X) {
  return B[0] = K[0] + W[0] * X, B[1] = K[1] + W[1] * X, B[2] = K[2] + W[2] * X, B[3] = K[3] + W[3] * X, B[4] = K[4] + W[4] * X, B[5] = K[5] + W[5] * X, B[6] = K[6] + W[6] * X, B[7] = K[7] + W[7] * X, B[8] = K[8] + W[8] * X, B[9] = K[9] + W[9] * X, B[10] = K[10] + W[10] * X, B[11] = K[11] + W[11] * X, B[12] = K[12] + W[12] * X, B[13] = K[13] + W[13] * X, B[14] = K[14] + W[14] * X, B[15] = K[15] + W[15] * X, B;
};
var zB = function(B, K) {
  return B[0] === K[0] && B[1] === K[1] && B[2] === K[2] && B[3] === K[3] && B[4] === K[4] && B[5] === K[5] && B[6] === K[6] && B[7] === K[7] && B[8] === K[8] && B[9] === K[9] && B[10] === K[10] && B[11] === K[11] && B[12] === K[12] && B[13] === K[13] && B[14] === K[14] && B[15] === K[15];
};
var rB = function(B, K) {
  var W = B[0], X = B[1], Y = B[2], $ = B[3], Q = B[4], Z = B[5], H = B[6], J = B[7], O = B[8], L = B[9], C = B[10], V = B[11], E = B[12], U = B[13], G = B[14], I = B[15], S = K[0], k = K[1], A = K[2], D = K[3], P = K[4], F = K[5], j = K[6], h = K[7], M = K[8], p = K[9], g = K[10], q = K[11], T = K[12], f = K[13], c = K[14], i = K[15];
  return Math.abs(W - S) <= N * Math.max(1, Math.abs(W), Math.abs(S)) && Math.abs(X - k) <= N * Math.max(1, Math.abs(X), Math.abs(k)) && Math.abs(Y - A) <= N * Math.max(1, Math.abs(Y), Math.abs(A)) && Math.abs($ - D) <= N * Math.max(1, Math.abs($), Math.abs(D)) && Math.abs(Q - P) <= N * Math.max(1, Math.abs(Q), Math.abs(P)) && Math.abs(Z - F) <= N * Math.max(1, Math.abs(Z), Math.abs(F)) && Math.abs(H - j) <= N * Math.max(1, Math.abs(H), Math.abs(j)) && Math.abs(J - h) <= N * Math.max(1, Math.abs(J), Math.abs(h)) && Math.abs(O - M) <= N * Math.max(1, Math.abs(O), Math.abs(M)) && Math.abs(L - p) <= N * Math.max(1, Math.abs(L), Math.abs(p)) && Math.abs(C - g) <= N * Math.max(1, Math.abs(C), Math.abs(g)) && Math.abs(V - q) <= N * Math.max(1, Math.abs(V), Math.abs(q)) && Math.abs(E - T) <= N * Math.max(1, Math.abs(E), Math.abs(T)) && Math.abs(U - f) <= N * Math.max(1, Math.abs(U), Math.abs(f)) && Math.abs(G - c) <= N * Math.max(1, Math.abs(G), Math.abs(c)) && Math.abs(I - i) <= N * Math.max(1, Math.abs(I), Math.abs(i));
};
var x = function() {
  var B = new R(3);
  if (R != Float32Array)
    B[0] = 0, B[1] = 0, B[2] = 0;
  return B;
};
var xB = function(B) {
  var K = new R(3);
  return K[0] = B[0], K[1] = B[1], K[2] = B[2], K;
};
var I0 = function(B) {
  var K = B[0], W = B[1], X = B[2];
  return Math.hypot(K, W, X);
};
var e = function(B, K, W) {
  var X = new R(3);
  return X[0] = B, X[1] = K, X[2] = W, X;
};
var eB = function(B, K) {
  return B[0] = K[0], B[1] = K[1], B[2] = K[2], B;
};
var bB = function(B, K, W, X) {
  return B[0] = K, B[1] = W, B[2] = X, B;
};
var uB = function(B, K, W) {
  return B[0] = K[0] + W[0], B[1] = K[1] + W[1], B[2] = K[2] + W[2], B;
};
var D0 = function(B, K, W) {
  return B[0] = K[0] - W[0], B[1] = K[1] - W[1], B[2] = K[2] - W[2], B;
};
var P0 = function(B, K, W) {
  return B[0] = K[0] * W[0], B[1] = K[1] * W[1], B[2] = K[2] * W[2], B;
};
var A0 = function(B, K, W) {
  return B[0] = K[0] / W[0], B[1] = K[1] / W[1], B[2] = K[2] / W[2], B;
};
var oB = function(B, K) {
  return B[0] = Math.ceil(K[0]), B[1] = Math.ceil(K[1]), B[2] = Math.ceil(K[2]), B;
};
var tB = function(B, K) {
  return B[0] = Math.floor(K[0]), B[1] = Math.floor(K[1]), B[2] = Math.floor(K[2]), B;
};
var aB = function(B, K, W) {
  return B[0] = Math.min(K[0], W[0]), B[1] = Math.min(K[1], W[1]), B[2] = Math.min(K[2], W[2]), B;
};
var BK = function(B, K, W) {
  return B[0] = Math.max(K[0], W[0]), B[1] = Math.max(K[1], W[1]), B[2] = Math.max(K[2], W[2]), B;
};
var KK = function(B, K) {
  return B[0] = Math.round(K[0]), B[1] = Math.round(K[1]), B[2] = Math.round(K[2]), B;
};
var WK = function(B, K, W) {
  return B[0] = K[0] * W, B[1] = K[1] * W, B[2] = K[2] * W, B;
};
var XK = function(B, K, W, X) {
  return B[0] = K[0] + W[0] * X, B[1] = K[1] + W[1] * X, B[2] = K[2] + W[2] * X, B;
};
var k0 = function(B, K) {
  var W = K[0] - B[0], X = K[1] - B[1], Y = K[2] - B[2];
  return Math.hypot(W, X, Y);
};
var N0 = function(B, K) {
  var W = K[0] - B[0], X = K[1] - B[1], Y = K[2] - B[2];
  return W * W + X * X + Y * Y;
};
var S0 = function(B) {
  var K = B[0], W = B[1], X = B[2];
  return K * K + W * W + X * X;
};
var YK = function(B, K) {
  return B[0] = -K[0], B[1] = -K[1], B[2] = -K[2], B;
};
var $K = function(B, K) {
  return B[0] = 1 / K[0], B[1] = 1 / K[1], B[2] = 1 / K[2], B;
};
var B0 = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = W * W + X * X + Y * Y;
  if ($ > 0)
    $ = 1 / Math.sqrt($);
  return B[0] = K[0] * $, B[1] = K[1] * $, B[2] = K[2] * $, B;
};
var b = function(B, K) {
  return B[0] * K[0] + B[1] * K[1] + B[2] * K[2];
};
var z = function(B, K, W) {
  var X = K[0], Y = K[1], $ = K[2], Q = W[0], Z = W[1], H = W[2];
  return B[0] = Y * H - $ * Z, B[1] = $ * Q - X * H, B[2] = X * Z - Y * Q, B;
};
var QK = function(B, K, W, X) {
  var Y = K[0], $ = K[1], Q = K[2];
  return B[0] = Y + X * (W[0] - Y), B[1] = $ + X * (W[1] - $), B[2] = Q + X * (W[2] - Q), B;
};
var ZK = function(B, K, W, X, Y, $) {
  var Q = $ * $, Z = Q * (2 * $ - 3) + 1, H = Q * ($ - 2) + $, J = Q * ($ - 1), O = Q * (3 - 2 * $);
  return B[0] = K[0] * Z + W[0] * H + X[0] * J + Y[0] * O, B[1] = K[1] * Z + W[1] * H + X[1] * J + Y[1] * O, B[2] = K[2] * Z + W[2] * H + X[2] * J + Y[2] * O, B;
};
var HK = function(B, K, W, X, Y, $) {
  var Q = 1 - $, Z = Q * Q, H = $ * $, J = Z * Q, O = 3 * $ * Z, L = 3 * H * Q, C = H * $;
  return B[0] = K[0] * J + W[0] * O + X[0] * L + Y[0] * C, B[1] = K[1] * J + W[1] * O + X[1] * L + Y[1] * C, B[2] = K[2] * J + W[2] * O + X[2] * L + Y[2] * C, B;
};
var JK = function(B, K) {
  K = K || 1;
  var W = v() * 2 * Math.PI, X = v() * 2 - 1, Y = Math.sqrt(1 - X * X) * K;
  return B[0] = Math.cos(W) * Y, B[1] = Math.sin(W) * Y, B[2] = X * K, B;
};
var OK = function(B, K, W) {
  var X = K[0], Y = K[1], $ = K[2], Q = W[3] * X + W[7] * Y + W[11] * $ + W[15];
  return Q = Q || 1, B[0] = (W[0] * X + W[4] * Y + W[8] * $ + W[12]) / Q, B[1] = (W[1] * X + W[5] * Y + W[9] * $ + W[13]) / Q, B[2] = (W[2] * X + W[6] * Y + W[10] * $ + W[14]) / Q, B;
};
var GK = function(B, K, W) {
  var X = K[0], Y = K[1], $ = K[2];
  return B[0] = X * W[0] + Y * W[3] + $ * W[6], B[1] = X * W[1] + Y * W[4] + $ * W[7], B[2] = X * W[2] + Y * W[5] + $ * W[8], B;
};
var LK = function(B, K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q = W[3], Z = K[0], H = K[1], J = K[2], O = Y * J - $ * H, L = $ * Z - X * J, C = X * H - Y * Z, V = Y * C - $ * L, E = $ * O - X * C, U = X * L - Y * O, G = Q * 2;
  return O *= G, L *= G, C *= G, V *= 2, E *= 2, U *= 2, B[0] = Z + O + V, B[1] = H + L + E, B[2] = J + C + U, B;
};
var VK = function(B, K, W, X) {
  var Y = [], $ = [];
  return Y[0] = K[0] - W[0], Y[1] = K[1] - W[1], Y[2] = K[2] - W[2], $[0] = Y[0], $[1] = Y[1] * Math.cos(X) - Y[2] * Math.sin(X), $[2] = Y[1] * Math.sin(X) + Y[2] * Math.cos(X), B[0] = $[0] + W[0], B[1] = $[1] + W[1], B[2] = $[2] + W[2], B;
};
var CK = function(B, K, W, X) {
  var Y = [], $ = [];
  return Y[0] = K[0] - W[0], Y[1] = K[1] - W[1], Y[2] = K[2] - W[2], $[0] = Y[2] * Math.sin(X) + Y[0] * Math.cos(X), $[1] = Y[1], $[2] = Y[2] * Math.cos(X) - Y[0] * Math.sin(X), B[0] = $[0] + W[0], B[1] = $[1] + W[1], B[2] = $[2] + W[2], B;
};
var UK = function(B, K, W, X) {
  var Y = [], $ = [];
  return Y[0] = K[0] - W[0], Y[1] = K[1] - W[1], Y[2] = K[2] - W[2], $[0] = Y[0] * Math.cos(X) - Y[1] * Math.sin(X), $[1] = Y[0] * Math.sin(X) + Y[1] * Math.cos(X), $[2] = Y[2], B[0] = $[0] + W[0], B[1] = $[1] + W[1], B[2] = $[2] + W[2], B;
};
var EK = function(B, K) {
  var W = B[0], X = B[1], Y = B[2], $ = K[0], Q = K[1], Z = K[2], H = Math.sqrt(W * W + X * X + Y * Y), J = Math.sqrt($ * $ + Q * Q + Z * Z), O = H * J, L = O && b(B, K) / O;
  return Math.acos(Math.min(Math.max(L, -1), 1));
};
var IK = function(B) {
  return B[0] = 0, B[1] = 0, B[2] = 0, B;
};
var DK = function(B) {
  return "vec3(" + B[0] + ", " + B[1] + ", " + B[2] + ")";
};
var PK = function(B, K) {
  return B[0] === K[0] && B[1] === K[1] && B[2] === K[2];
};
var AK = function(B, K) {
  var W = B[0], X = B[1], Y = B[2], $ = K[0], Q = K[1], Z = K[2];
  return Math.abs(W - $) <= N * Math.max(1, Math.abs(W), Math.abs($)) && Math.abs(X - Q) <= N * Math.max(1, Math.abs(X), Math.abs(Q)) && Math.abs(Y - Z) <= N * Math.max(1, Math.abs(Y), Math.abs(Z));
};
var hK = function() {
  var B = new R(4);
  if (R != Float32Array)
    B[0] = 0, B[1] = 0, B[2] = 0, B[3] = 0;
  return B;
};
var T0 = function(B) {
  var K = new R(4);
  return K[0] = B[0], K[1] = B[1], K[2] = B[2], K[3] = B[3], K;
};
var _0 = function(B, K, W, X) {
  var Y = new R(4);
  return Y[0] = B, Y[1] = K, Y[2] = W, Y[3] = X, Y;
};
var R0 = function(B, K) {
  return B[0] = K[0], B[1] = K[1], B[2] = K[2], B[3] = K[3], B;
};
var j0 = function(B, K, W, X, Y) {
  return B[0] = K, B[1] = W, B[2] = X, B[3] = Y, B;
};
var h0 = function(B, K, W) {
  return B[0] = K[0] + W[0], B[1] = K[1] + W[1], B[2] = K[2] + W[2], B[3] = K[3] + W[3], B;
};
var M0 = function(B, K, W) {
  return B[0] = K[0] * W, B[1] = K[1] * W, B[2] = K[2] * W, B[3] = K[3] * W, B;
};
var F0 = function(B) {
  var K = B[0], W = B[1], X = B[2], Y = B[3];
  return Math.hypot(K, W, X, Y);
};
var p0 = function(B) {
  var K = B[0], W = B[1], X = B[2], Y = B[3];
  return K * K + W * W + X * X + Y * Y;
};
var g0 = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3], Q = W * W + X * X + Y * Y + $ * $;
  if (Q > 0)
    Q = 1 / Math.sqrt(Q);
  return B[0] = W * Q, B[1] = X * Q, B[2] = Y * Q, B[3] = $ * Q, B;
};
var q0 = function(B, K) {
  return B[0] * K[0] + B[1] * K[1] + B[2] * K[2] + B[3] * K[3];
};
var w0 = function(B, K, W, X) {
  var Y = K[0], $ = K[1], Q = K[2], Z = K[3];
  return B[0] = Y + X * (W[0] - Y), B[1] = $ + X * (W[1] - $), B[2] = Q + X * (W[2] - Q), B[3] = Z + X * (W[3] - Z), B;
};
var f0 = function(B, K) {
  return B[0] === K[0] && B[1] === K[1] && B[2] === K[2] && B[3] === K[3];
};
var d0 = function(B, K) {
  var W = B[0], X = B[1], Y = B[2], $ = B[3], Q = K[0], Z = K[1], H = K[2], J = K[3];
  return Math.abs(W - Q) <= N * Math.max(1, Math.abs(W), Math.abs(Q)) && Math.abs(X - Z) <= N * Math.max(1, Math.abs(X), Math.abs(Z)) && Math.abs(Y - H) <= N * Math.max(1, Math.abs(Y), Math.abs(H)) && Math.abs($ - J) <= N * Math.max(1, Math.abs($), Math.abs(J));
};
var W0 = function() {
  var B = new R(4);
  if (R != Float32Array)
    B[0] = 0, B[1] = 0, B[2] = 0;
  return B[3] = 1, B;
};
var FK = function(B) {
  return B[0] = 0, B[1] = 0, B[2] = 0, B[3] = 1, B;
};
var v0 = function(B, K, W) {
  W = W * 0.5;
  var X = Math.sin(W);
  return B[0] = X * K[0], B[1] = X * K[1], B[2] = X * K[2], B[3] = Math.cos(W), B;
};
var pK = function(B, K) {
  var W = Math.acos(K[3]) * 2, X = Math.sin(W / 2);
  if (X > N)
    B[0] = K[0] / X, B[1] = K[1] / X, B[2] = K[2] / X;
  else
    B[0] = 1, B[1] = 0, B[2] = 0;
  return W;
};
var gK = function(B, K) {
  var W = z0(B, K);
  return Math.acos(2 * W * W - 1);
};
var n0 = function(B, K, W) {
  var X = K[0], Y = K[1], $ = K[2], Q = K[3], Z = W[0], H = W[1], J = W[2], O = W[3];
  return B[0] = X * O + Q * Z + Y * J - $ * H, B[1] = Y * O + Q * H + $ * Z - X * J, B[2] = $ * O + Q * J + X * H - Y * Z, B[3] = Q * O - X * Z - Y * H - $ * J, B;
};
var qK = function(B, K, W) {
  W *= 0.5;
  var X = K[0], Y = K[1], $ = K[2], Q = K[3], Z = Math.sin(W), H = Math.cos(W);
  return B[0] = X * H + Q * Z, B[1] = Y * H + $ * Z, B[2] = $ * H - Y * Z, B[3] = Q * H - X * Z, B;
};
var wK = function(B, K, W) {
  W *= 0.5;
  var X = K[0], Y = K[1], $ = K[2], Q = K[3], Z = Math.sin(W), H = Math.cos(W);
  return B[0] = X * H - $ * Z, B[1] = Y * H + Q * Z, B[2] = $ * H + X * Z, B[3] = Q * H - Y * Z, B;
};
var fK = function(B, K, W) {
  W *= 0.5;
  var X = K[0], Y = K[1], $ = K[2], Q = K[3], Z = Math.sin(W), H = Math.cos(W);
  return B[0] = X * H + Y * Z, B[1] = Y * H - X * Z, B[2] = $ * H + Q * Z, B[3] = Q * H - $ * Z, B;
};
var dK = function(B, K) {
  var W = K[0], X = K[1], Y = K[2];
  return B[0] = W, B[1] = X, B[2] = Y, B[3] = Math.sqrt(Math.abs(1 - W * W - X * X - Y * Y)), B;
};
var l0 = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3], Q = Math.sqrt(W * W + X * X + Y * Y), Z = Math.exp($), H = Q > 0 ? Z * Math.sin(Q) / Q : 0;
  return B[0] = W * H, B[1] = X * H, B[2] = Y * H, B[3] = Z * Math.cos(Q), B;
};
var c0 = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3], Q = Math.sqrt(W * W + X * X + Y * Y), Z = Q > 0 ? Math.atan2(Q, $) / Q : 0;
  return B[0] = W * Z, B[1] = X * Z, B[2] = Y * Z, B[3] = 0.5 * Math.log(W * W + X * X + Y * Y + $ * $), B;
};
var vK = function(B, K, W) {
  return c0(B, K), y0(B, B, W), l0(B, B), B;
};
var u = function(B, K, W, X) {
  var Y = K[0], $ = K[1], Q = K[2], Z = K[3], H = W[0], J = W[1], O = W[2], L = W[3], C, V, E, U, G;
  if (V = Y * H + $ * J + Q * O + Z * L, V < 0)
    V = -V, H = -H, J = -J, O = -O, L = -L;
  if (1 - V > N)
    C = Math.acos(V), E = Math.sin(C), U = Math.sin((1 - X) * C) / E, G = Math.sin(X * C) / E;
  else
    U = 1 - X, G = X;
  return B[0] = U * Y + G * H, B[1] = U * $ + G * J, B[2] = U * Q + G * O, B[3] = U * Z + G * L, B;
};
var nK = function(B) {
  var K = v(), W = v(), X = v(), Y = Math.sqrt(1 - K), $ = Math.sqrt(K);
  return B[0] = Y * Math.sin(2 * Math.PI * W), B[1] = Y * Math.cos(2 * Math.PI * W), B[2] = $ * Math.sin(2 * Math.PI * X), B[3] = $ * Math.cos(2 * Math.PI * X), B;
};
var lK = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3], Q = W * W + X * X + Y * Y + $ * $, Z = Q ? 1 / Q : 0;
  return B[0] = -W * Z, B[1] = -X * Z, B[2] = -Y * Z, B[3] = $ * Z, B;
};
var cK = function(B, K) {
  return B[0] = -K[0], B[1] = -K[1], B[2] = -K[2], B[3] = K[3], B;
};
var i0 = function(B, K) {
  var W = K[0] + K[4] + K[8], X;
  if (W > 0)
    X = Math.sqrt(W + 1), B[3] = 0.5 * X, X = 0.5 / X, B[0] = (K[5] - K[7]) * X, B[1] = (K[6] - K[2]) * X, B[2] = (K[1] - K[3]) * X;
  else {
    var Y = 0;
    if (K[4] > K[0])
      Y = 1;
    if (K[8] > K[Y * 3 + Y])
      Y = 2;
    var $ = (Y + 1) % 3, Q = (Y + 2) % 3;
    X = Math.sqrt(K[Y * 3 + Y] - K[$ * 3 + $] - K[Q * 3 + Q] + 1), B[Y] = 0.5 * X, X = 0.5 / X, B[3] = (K[$ * 3 + Q] - K[Q * 3 + $]) * X, B[$] = (K[$ * 3 + Y] + K[Y * 3 + $]) * X, B[Q] = (K[Q * 3 + Y] + K[Y * 3 + Q]) * X;
  }
  return B;
};
var iK = function(B, K, W, X) {
  var Y = 0.5 * Math.PI / 180;
  K *= Y, W *= Y, X *= Y;
  var $ = Math.sin(K), Q = Math.cos(K), Z = Math.sin(W), H = Math.cos(W), J = Math.sin(X), O = Math.cos(X);
  return B[0] = $ * H * O - Q * Z * J, B[1] = Q * Z * O + $ * H * J, B[2] = Q * H * J - $ * Z * O, B[3] = Q * H * O + $ * Z * J, B;
};
var yK = function(B) {
  return "quat(" + B[0] + ", " + B[1] + ", " + B[2] + ", " + B[3] + ")";
};
var t = function(B, K, W, X) {
  return X[0] = B, X[1] = K, X[2] = W, X;
};
var Q0 = function(B, K) {
  const W = B.getMatrix();
  K[0] = W[12], K[1] = W[13], K[2] = W[14];
};
var BB = Object.defineProperty;
var a = (B, K) => {
  for (var W in K)
    BB(B, W, { get: K[W], enumerable: true, configurable: true, set: (X) => K[W] = () => X });
};
var N = 0.000001;
var R = typeof Float32Array !== "undefined" ? Float32Array : Array;
var v = Math.random;
var GW = Math.PI / 180;
if (!Math.hypot)
  Math.hypot = function() {
    var B = 0, K = arguments.length;
    while (K--)
      B += arguments[K] * arguments[K];
    return Math.sqrt(B);
  };
var _ = {};
a(_, { transpose: () => {
  {
    return HB;
  }
}, translate: () => {
  {
    return LB;
  }
}, targetTo: () => {
  {
    return vB;
  }
}, subtract: () => {
  {
    return E0;
  }
}, sub: () => {
  {
    return mB;
  }
}, str: () => {
  {
    return nB;
  }
}, set: () => {
  {
    return ZB;
  }
}, scale: () => {
  {
    return VB;
  }
}, rotateZ: () => {
  {
    return IB;
  }
}, rotateY: () => {
  {
    return EB;
  }
}, rotateX: () => {
  {
    return UB;
  }
}, rotate: () => {
  {
    return CB;
  }
}, perspectiveZO: () => {
  {
    return gB;
  }
}, perspectiveNO: () => {
  {
    return C0;
  }
}, perspectiveFromFieldOfView: () => {
  {
    return qB;
  }
}, perspective: () => {
  {
    return pB;
  }
}, orthoZO: () => {
  {
    return fB;
  }
}, orthoNO: () => {
  {
    return U0;
  }
}, ortho: () => {
  {
    return wB;
  }
}, multiplyScalarAndAdd: () => {
  {
    return yB;
  }
}, multiplyScalar: () => {
  {
    return iB;
  }
}, multiply: () => {
  {
    return G0;
  }
}, mul: () => {
  {
    return sB;
  }
}, lookAt: () => {
  {
    return dB;
  }
}, invert: () => {
  {
    return JB;
  }
}, identity: () => {
  {
    return O0;
  }
}, getTranslation: () => {
  {
    return _B;
  }
}, getScaling: () => {
  {
    return V0;
  }
}, getRotation: () => {
  {
    return RB;
  }
}, frustum: () => {
  {
    return FB;
  }
}, fromZRotation: () => {
  {
    return SB;
  }
}, fromYRotation: () => {
  {
    return NB;
  }
}, fromXRotation: () => {
  {
    return kB;
  }
}, fromValues: () => {
  {
    return QB;
  }
}, fromTranslation: () => {
  {
    return DB;
  }
}, fromScaling: () => {
  {
    return PB;
  }
}, fromRotationTranslationScaleOrigin: () => {
  {
    return hB;
  }
}, fromRotationTranslationScale: () => {
  {
    return jB;
  }
}, fromRotationTranslation: () => {
  {
    return L0;
  }
}, fromRotation: () => {
  {
    return AB;
  }
}, fromQuat2: () => {
  {
    return TB;
  }
}, fromQuat: () => {
  {
    return MB;
  }
}, frob: () => {
  {
    return lB;
  }
}, exactEquals: () => {
  {
    return zB;
  }
}, equals: () => {
  {
    return rB;
  }
}, determinant: () => {
  {
    return GB;
  }
}, create: () => {
  {
    return XB;
  }
}, copy: () => {
  {
    return $B;
  }
}, clone: () => {
  {
    return YB;
  }
}, adjoint: () => {
  {
    return OB;
  }
}, add: () => {
  {
    return cB;
  }
} });
var pB = C0;
var wB = U0;
var sB = G0;
var mB = E0;
var s = {};
a(s, { str: () => {
  {
    return yK;
  }
}, squaredLength: () => {
  {
    return s0;
  }
}, sqrLen: () => {
  {
    return oK;
  }
}, sqlerp: () => {
  {
    return KW;
  }
}, slerp: () => {
  {
    return u;
  }
}, setAxisAngle: () => {
  {
    return v0;
  }
}, setAxes: () => {
  {
    return WW;
  }
}, set: () => {
  {
    return mK;
  }
}, scale: () => {
  {
    return y0;
  }
}, rotationTo: () => {
  {
    return BW;
  }
}, rotateZ: () => {
  {
    return fK;
  }
}, rotateY: () => {
  {
    return wK;
  }
}, rotateX: () => {
  {
    return qK;
  }
}, random: () => {
  {
    return nK;
  }
}, pow: () => {
  {
    return vK;
  }
}, normalize: () => {
  {
    return X0;
  }
}, multiply: () => {
  {
    return n0;
  }
}, mul: () => {
  {
    return eK;
  }
}, ln: () => {
  {
    return c0;
  }
}, lerp: () => {
  {
    return bK;
  }
}, length: () => {
  {
    return r0;
  }
}, len: () => {
  {
    return uK;
  }
}, invert: () => {
  {
    return lK;
  }
}, identity: () => {
  {
    return FK;
  }
}, getAxisAngle: () => {
  {
    return pK;
  }
}, getAngle: () => {
  {
    return gK;
  }
}, fromValues: () => {
  {
    return rK;
  }
}, fromMat3: () => {
  {
    return i0;
  }
}, fromEuler: () => {
  {
    return iK;
  }
}, exp: () => {
  {
    return l0;
  }
}, exactEquals: () => {
  {
    return tK;
  }
}, equals: () => {
  {
    return aK;
  }
}, dot: () => {
  {
    return z0;
  }
}, create: () => {
  {
    return W0;
  }
}, copy: () => {
  {
    return sK;
  }
}, conjugate: () => {
  {
    return cK;
  }
}, clone: () => {
  {
    return zK;
  }
}, calculateW: () => {
  {
    return dK;
  }
}, add: () => {
  {
    return xK;
  }
} });
var r = {};
a(r, { zero: () => {
  {
    return IK;
  }
}, transformQuat: () => {
  {
    return LK;
  }
}, transformMat4: () => {
  {
    return OK;
  }
}, transformMat3: () => {
  {
    return GK;
  }
}, subtract: () => {
  {
    return D0;
  }
}, sub: () => {
  {
    return kK;
  }
}, str: () => {
  {
    return DK;
  }
}, squaredLength: () => {
  {
    return S0;
  }
}, squaredDistance: () => {
  {
    return N0;
  }
}, sqrLen: () => {
  {
    return RK;
  }
}, sqrDist: () => {
  {
    return _K;
  }
}, set: () => {
  {
    return bB;
  }
}, scaleAndAdd: () => {
  {
    return XK;
  }
}, scale: () => {
  {
    return WK;
  }
}, round: () => {
  {
    return KK;
  }
}, rotateZ: () => {
  {
    return UK;
  }
}, rotateY: () => {
  {
    return CK;
  }
}, rotateX: () => {
  {
    return VK;
  }
}, random: () => {
  {
    return JK;
  }
}, normalize: () => {
  {
    return B0;
  }
}, negate: () => {
  {
    return YK;
  }
}, multiply: () => {
  {
    return P0;
  }
}, mul: () => {
  {
    return NK;
  }
}, min: () => {
  {
    return aB;
  }
}, max: () => {
  {
    return BK;
  }
}, lerp: () => {
  {
    return QK;
  }
}, length: () => {
  {
    return I0;
  }
}, len: () => {
  {
    return K0;
  }
}, inverse: () => {
  {
    return $K;
  }
}, hermite: () => {
  {
    return ZK;
  }
}, fromValues: () => {
  {
    return e;
  }
}, forEach: () => {
  {
    return jK;
  }
}, floor: () => {
  {
    return tB;
  }
}, exactEquals: () => {
  {
    return PK;
  }
}, equals: () => {
  {
    return AK;
  }
}, dot: () => {
  {
    return b;
  }
}, divide: () => {
  {
    return A0;
  }
}, div: () => {
  {
    return SK;
  }
}, distance: () => {
  {
    return k0;
  }
}, dist: () => {
  {
    return TK;
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
    return eB;
  }
}, clone: () => {
  {
    return xB;
  }
}, ceil: () => {
  {
    return oB;
  }
}, bezier: () => {
  {
    return HK;
  }
}, angle: () => {
  {
    return EK;
  }
}, add: () => {
  {
    return uB;
  }
} });
var kK = D0;
var NK = P0;
var SK = A0;
var TK = k0;
var _K = N0;
var K0 = I0;
var RK = S0;
var jK = function() {
  var B = x();
  return function(K, W, X, Y, $, Q) {
    var Z, H;
    if (!W)
      W = 3;
    if (!X)
      X = 0;
    if (Y)
      H = Math.min(Y * W + X, K.length);
    else
      H = K.length;
    for (Z = X;Z < H; Z += W)
      B[0] = K[Z], B[1] = K[Z + 1], B[2] = K[Z + 2], $(B, B, Q), K[Z] = B[0], K[Z + 1] = B[1], K[Z + 2] = B[2];
    return K;
  };
}();
var LW = function() {
  var B = hK();
  return function(K, W, X, Y, $, Q) {
    var Z, H;
    if (!W)
      W = 4;
    if (!X)
      X = 0;
    if (Y)
      H = Math.min(Y * W + X, K.length);
    else
      H = K.length;
    for (Z = X;Z < H; Z += W)
      B[0] = K[Z], B[1] = K[Z + 1], B[2] = K[Z + 2], B[3] = K[Z + 3], $(B, B, Q), K[Z] = B[0], K[Z + 1] = B[1], K[Z + 2] = B[2], K[Z + 3] = B[3];
    return K;
  };
}();
var zK = T0;
var rK = _0;
var sK = R0;
var mK = j0;
var xK = h0;
var eK = n0;
var y0 = M0;
var z0 = q0;
var bK = w0;
var r0 = F0;
var uK = r0;
var s0 = p0;
var oK = s0;
var X0 = g0;
var tK = f0;
var aK = d0;
var BW = function() {
  var B = x(), K = e(1, 0, 0), W = e(0, 1, 0);
  return function(X, Y, $) {
    var Q = b(Y, $);
    if (Q < -0.999999) {
      if (z(B, K, Y), K0(B) < 0.000001)
        z(B, W, Y);
      return B0(B, B), v0(X, B, Math.PI), X;
    } else if (Q > 0.999999)
      return X[0] = 0, X[1] = 0, X[2] = 0, X[3] = 1, X;
    else
      return z(B, Y, $), X[0] = B[0], X[1] = B[1], X[2] = B[2], X[3] = 1 + Q, X0(X, X);
  };
}();
var KW = function() {
  var B = W0(), K = W0();
  return function(W, X, Y, $, Q, Z) {
    return u(B, X, Q, Z), u(K, Y, $, Z), u(W, B, K, 2 * Z * (1 - Z)), W;
  };
}();
var WW = function() {
  var B = J0();
  return function(K, W, X, Y) {
    return B[0] = X[0], B[3] = X[1], B[6] = X[2], B[1] = Y[0], B[4] = Y[1], B[7] = Y[2], B[2] = -W[0], B[5] = -W[1], B[8] = -W[2], X0(K, i0(K, B));
  };
}();

class n {
  d;
  listeners = new Set;
  constructor(B) {
    this.elem = B;
  }
  addChangeListener(B) {
    return this.listeners.add(B), this;
  }
  removeChangeListener(B) {
    this.listeners.delete(B);
  }
  onChange() {
    for (let B of this.listeners)
      B.onChange(this.elem);
  }
}
var XW = Math.PI / 90;
var Y0 = [0, 0, 0];
var m0 = _.create();
var x0 = _.create();
var o = s.create();

class m {
  static HIDDEN = m.create().scale(0, 0, 0);
  static IDENTITY = m.create();
  #B = Float32Array.from(_.create());
  #K = new n(this);
  constructor() {
    this.identity();
  }
  addChangeListener(B) {
    return this.#K.addChangeListener(B), this;
  }
  removeChangeListener(B) {
    this.#K.removeChangeListener(B);
  }
  static create() {
    return new m;
  }
  copy(B) {
    return _.copy(this.#B, B.getMatrix()), this.#K.onChange(), this;
  }
  identity() {
    return _.identity(this.#B), this.#K.onChange(), this;
  }
  invert(B) {
    return _.invert(this.#B, B?.getMatrix() ?? this.getMatrix()), this.#K.onChange(), this;
  }
  multiply(B) {
    return _.multiply(this.#B, this.#B, B.getMatrix()), this.#K.onChange(), this;
  }
  multiply2(B, K) {
    return _.multiply(this.#B, B.getMatrix(), K.getMatrix()), this.#K.onChange(), this;
  }
  multiply3(B, K, W) {
    return this.multiply2(B, K), this.multiply(W), this;
  }
  translate(B, K, W) {
    const X = Y0;
    return X[0] = B, X[1] = K, X[2] = W, this.move(X);
  }
  move(B) {
    return _.translate(this.#B, this.#B, B), this.#K.onChange(), this;
  }
  rotateX(B) {
    return _.rotateX(this.#B, this.#B, B), this.#K.onChange(), this;
  }
  rotateY(B) {
    return _.rotateY(this.#B, this.#B, B), this.#K.onChange(), this;
  }
  rotateZ(B) {
    return _.rotateZ(this.#B, this.#B, B), this.#K.onChange(), this;
  }
  setXRotation(B) {
    return _.fromXRotation(this.getMatrix(), B), this.#K.onChange(), this;
  }
  setYRotation(B) {
    return _.fromYRotation(this.getMatrix(), B), this.#K.onChange(), this;
  }
  scale(B, K, W) {
    return _.scale(this.#B, this.#B, [B, K ?? B, W ?? B]), this.#K.onChange(), this;
  }
  perspective(B, K, W, X) {
    return _.perspective(this.#B, B * XW, K, W, X), this.#K.onChange(), this;
  }
  ortho(B, K, W, X, Y, $) {
    return _.ortho(this.#B, B, K, W, X, Y, $), this.#K.onChange(), this;
  }
  combine(B, K, W = 0.5) {
    return _.multiplyScalar(m0, B.getMatrix(), 1 - W), _.multiplyScalar(x0, K.getMatrix(), W), _.add(this.#B, m0, x0), this.#K.onChange(), this;
  }
  static getMoveVector(B, K, W, X) {
    const Y = Y0;
    if (Y[0] = B, Y[1] = K, Y[2] = W, X)
      _.getRotation(o, X.getMatrix()), s.invert(o, o), r.transformQuat(Y, Y, o);
    return Y;
  }
  getPosition() {
    const B = Y0;
    return B[0] = this.#B[12], B[1] = this.#B[13], B[2] = this.#B[14], B;
  }
  setVector(B) {
    return this.setPosition(B[0], B[1], B[2]);
  }
  setPosition(B, K, W) {
    return this.#B[12] = B, this.#B[13] = K, this.#B[14] = W, this.#K.onChange(), this;
  }
  getMatrix() {
    return this.#B;
  }
}
var d = m;

class e0 {
  q;
  w;
  #B;
  #K = false;
  #W = 0;
  #X;
  #Y;
  constructor(B, K, W) {
    this.getValue = K, this.apply = W, this.#X = B, this.#B = this.getValue(B);
  }
  set element(B) {
    this.#X = B, this.#B = this.getValue(B), this.#Y = undefined;
  }
  setGoal(B, K, W) {
    if (this.#Y && this.#Y !== W)
      return;
    if (this.#B !== B || this.#W !== K)
      this.#W = K, this.#B = B, this.#Y = W, this.#K = true;
  }
  get goal() {
    return this.#B;
  }
  update(B) {
    if (this.#K) {
      const K = this.getValue(this.#X), W = this.goal - K, X = Math.min(Math.abs(W), this.#W * B);
      if (X <= 0.01)
        this.apply(this.#X, this.goal), this.#K = false, this.#Y?.onRelease?.(), this.#Y = undefined;
      else
        this.apply(this.#X, K + X * Math.sign(W));
    }
    return this.#K;
  }
}

class b0 {
  i;
  f;
  warningLimit = 50000;
  #B = new Set;
  #K = [];
  constructor(B, K) {
    this.initCall = B, this.onRecycle = K;
  }
  create(...B) {
    const K = this.#K.pop();
    if (K)
      return this.#B.add(K), this.initCall(K, ...B);
    const W = this.initCall(undefined, ...B);
    return this.#B.add(W), this.#X(), W;
  }
  recycle(B) {
    this.#B.delete(B), this.#W(B);
  }
  recycleAll() {
    for (let B of this.#B)
      this.#W(B);
    this.#B.clear();
  }
  clear() {
    this.#K.length = 0, this.#B.clear();
  }
  countObjectsInExistence() {
    return this.#B.size + this.#K.length;
  }
  #W(B) {
    this.#K.push(B), this.onRecycle?.(B);
  }
  #X() {
    if (this.countObjectsInExistence() === this.warningLimit)
      console.warn("ObjectPool already created", this.#B.size + this.#K.length, "in", this.constructor.name);
  }
}

class u0 extends b0 {
  constructor() {
    super((B, K) => {
      if (!B)
        return new e0(K, (W) => W.valueOf(), (W, X) => W.setValue(X));
      return B.element = K, B;
    });
  }
}
var YW = new u0;

class l {
  q;
  w;
  #B = 0;
  #K;
  constructor(B = 0, K, W = YW) {
    this.onChange = K, this.pool = W, this.#B = B;
  }
  valueOf() {
    return this.#B;
  }
  setValue(B) {
    if (B !== this.#B)
      this.#B = B, this.onChange?.(this.#B);
    return this;
  }
  addValue(B) {
    return this.setValue(this.#B + B), this;
  }
  update(B) {
    if (this.#K) {
      const K = !!this.#K.update(B);
      if (!K)
        this.pool.recycle(this.#K), this.#K = undefined;
      return K;
    }
    return false;
  }
  refresh({ deltaTime: B, stopUpdate: K }) {
    if (!this.update(B))
      K();
  }
  progressTowards(B, K, W, X) {
    if (!this.#K)
      this.#K = this.pool.create(this);
    if (this.#K.setGoal(B, K, W), X)
      X.loop(this, undefined);
  }
  get goal() {
    return this.#K?.goal ?? this.valueOf();
  }
}
var w;
(function(X) {
  X[X["AT_POSITION"] = 0] = "AT_POSITION";
  X[X["MOVED"] = 1] = "MOVED";
  X[X["BLOCKED"] = 2] = "BLOCKED";
})(w || (w = {}));
var $0 = function(B, K) {
  if (B) {
    const W = B.length.valueOf();
    for (let X = 0;X < W; X++) {
      const Y = B.at(X);
      if (Y !== undefined && K(Y, X))
        return true;
    }
  }
  return false;
};

class t0 {
  #B = d.create().setPosition(0, 0, 0);
  #K = [0, 0, 0];
  #W = new n(this);
  position = [0, 0, 0];
  blockers;
  constructor({ blockers: B } = {}) {
    this.blockers = B, this.#B.addChangeListener({ onChange: () => {
      Q0(this.#B, this.position), this.#W.onChange();
    } });
  }
  addChangeListener(B) {
    return this.#W.addChangeListener(B), this;
  }
  removeChangeListener(B) {
    this.#W.removeChangeListener(B);
  }
  moveBy(B, K, W, X) {
    const Y = d.getMoveVector(B, K, W, X), $ = $0(this.blockers, (Q) => Q.isBlocked(t(this.position[0] + Y[0], this.position[1] + Y[1], this.position[2] + Y[2], this.#K), this.position));
    if (!$)
      if (Y[0] || Y[1] || Y[2])
        this.#B.move(Y);
      else
        return w.AT_POSITION;
    return $ ? w.BLOCKED : w.MOVED;
  }
  moveTo(B, K, W) {
    if (this.position[0] === B && this.position[1] === K && this.position[2] === W)
      return w.AT_POSITION;
    const X = $0(this.blockers, (Y) => Y.isBlocked(t(B, K, W, this.#K), this.position));
    if (!X) {
      const [Y, $, Q] = this.#B.getPosition();
      if (Y !== B || $ !== K || Q !== W)
        this.#B.setPosition(B, K, W);
    }
    return X ? w.BLOCKED : w.MOVED;
  }
  movedTo(B, K, W) {
    return this.moveTo(B, K, W), this;
  }
  moveTowards(B, K, W, X = 0.1) {
    const Y = this.position, $ = B - Y[0], Q = K - Y[1], Z = W - Y[2], H = Math.sqrt($ * $ + Q * Q + Z * Z);
    if (H > 0.01) {
      const J = Math.min(H, X) / H;
      return this.moveBy($ * J, Q * J, Z * J);
    } else
      return this.moveTo(B, K, W);
  }
  attemptMoveTowards(B, K, W, X = 0.1) {
    let Y = this.moveTowards(B, K, W, X);
    if (Y === w.BLOCKED && B !== this.position[0])
      Y = this.moveTowards(B, this.position[1], this.position[2], X);
    if (Y === w.BLOCKED && K !== this.position[1])
      Y = this.moveTowards(this.position[0], K, this.position[2], X);
    if (Y === w.BLOCKED && W)
      Y = this.moveTowards(this.position[0], this.position[1], W, X);
    return Y;
  }
  getMatrix() {
    return this.#B.getMatrix();
  }
}
var ZW = 1;
var HW = 1;

class a0 {
  #B = d.create();
  #K = d.create();
  #W = d.create();
  #X = [0, 0];
  #Y = new n(this);
  perspective;
  zoom;
  constructor() {
    const B = { onChange: () => {
      this.#B.combine(this.#W, this.#K, this.perspective.valueOf());
    } };
    this.perspective = new l(ZW, B.onChange), this.zoom = new l(HW, (K) => {
      this.configure(this.#X, K);
    }), this.#K.addChangeListener(B), this.#W.addChangeListener(B), this.#B.addChangeListener(this.#Y);
  }
  addChangeListener(B) {
    return this.#Y.addChangeListener(B), this;
  }
  removeChangeListener(B) {
    this.#Y.removeChangeListener(B);
  }
  configPerspectiveMatrix(B, K, W, X) {
    this.#K.perspective(B, K, W, X);
  }
  configOrthoMatrix(B, K, W, X) {
    this.#W.ortho(-B / 2, B / 2, -K / 2, K / 2, W, X);
  }
  configure(B, K, W = 0.5, X = 1e4) {
    if (!K)
      K = this.zoom.valueOf();
    this.#X[0] = B[0], this.#X[1] = B[1];
    const Y = this.#X[0] / this.#X[1], $ = 45 / Math.sqrt(K);
    this.configPerspectiveMatrix($, Y, Math.max(W, 0.00001), X), this.configOrthoMatrix(Y / K / K, 1 / K / K, -X, X);
  }
  getMatrix() {
    return this.#B.getMatrix();
  }
}
export {
  d as Matrix
};
