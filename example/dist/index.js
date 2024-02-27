// /Users/vincent/dok-matrix/example/node_modules/dok-matrix/dist/index.js
var G0 = function() {
  var B = new _(9);
  if (_ != Float32Array)
    B[1] = 0, B[2] = 0, B[3] = 0, B[5] = 0, B[6] = 0, B[7] = 0;
  return B[0] = 1, B[4] = 1, B[8] = 1, B;
};
var XB = function() {
  var B = new _(16);
  if (_ != Float32Array)
    B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0;
  return B[0] = 1, B[5] = 1, B[10] = 1, B[15] = 1, B;
};
var YB = function(B) {
  var K = new _(16);
  return K[0] = B[0], K[1] = B[1], K[2] = B[2], K[3] = B[3], K[4] = B[4], K[5] = B[5], K[6] = B[6], K[7] = B[7], K[8] = B[8], K[9] = B[9], K[10] = B[10], K[11] = B[11], K[12] = B[12], K[13] = B[13], K[14] = B[14], K[15] = B[15], K;
};
var $B = function(B, K) {
  return B[0] = K[0], B[1] = K[1], B[2] = K[2], B[3] = K[3], B[4] = K[4], B[5] = K[5], B[6] = K[6], B[7] = K[7], B[8] = K[8], B[9] = K[9], B[10] = K[10], B[11] = K[11], B[12] = K[12], B[13] = K[13], B[14] = K[14], B[15] = K[15], B;
};
var QB = function(B, K, W, X, Y, $, Q, Z, O, G, H, L, C, V, E, U) {
  var J = new _(16);
  return J[0] = B, J[1] = K, J[2] = W, J[3] = X, J[4] = Y, J[5] = $, J[6] = Q, J[7] = Z, J[8] = O, J[9] = G, J[10] = H, J[11] = L, J[12] = C, J[13] = V, J[14] = E, J[15] = U, J;
};
var ZB = function(B, K, W, X, Y, $, Q, Z, O, G, H, L, C, V, E, U, J) {
  return B[0] = K, B[1] = W, B[2] = X, B[3] = Y, B[4] = $, B[5] = Q, B[6] = Z, B[7] = O, B[8] = G, B[9] = H, B[10] = L, B[11] = C, B[12] = V, B[13] = E, B[14] = U, B[15] = J, B;
};
var H0 = function(B) {
  return B[0] = 1, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = 1, B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[10] = 1, B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0, B[15] = 1, B;
};
var OB = function(B, K) {
  if (B === K) {
    var W = K[1], X = K[2], Y = K[3], $ = K[6], Q = K[7], Z = K[11];
    B[1] = K[4], B[2] = K[8], B[3] = K[12], B[4] = W, B[6] = K[9], B[7] = K[13], B[8] = X, B[9] = $, B[11] = K[14], B[12] = Y, B[13] = Q, B[14] = Z;
  } else
    B[0] = K[0], B[1] = K[4], B[2] = K[8], B[3] = K[12], B[4] = K[1], B[5] = K[5], B[6] = K[9], B[7] = K[13], B[8] = K[2], B[9] = K[6], B[10] = K[10], B[11] = K[14], B[12] = K[3], B[13] = K[7], B[14] = K[11], B[15] = K[15];
  return B;
};
var GB = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3], Q = K[4], Z = K[5], O = K[6], G = K[7], H = K[8], L = K[9], C = K[10], V = K[11], E = K[12], U = K[13], J = K[14], I = K[15], R = W * Z - X * Q, k = W * O - Y * Q, A = W * G - $ * Q, D = X * O - Y * Z, P = X * G - $ * Z, F = Y * G - $ * O, j = H * U - L * E, h = H * J - C * E, M = H * I - V * E, p = L * J - C * U, g = L * I - V * U, q = C * I - V * J, S = R * q - k * g + A * p + D * M - P * h + F * j;
  if (!S)
    return null;
  return S = 1 / S, B[0] = (Z * q - O * g + G * p) * S, B[1] = (Y * g - X * q - $ * p) * S, B[2] = (U * F - J * P + I * D) * S, B[3] = (C * P - L * F - V * D) * S, B[4] = (O * M - Q * q - G * h) * S, B[5] = (W * q - Y * M + $ * h) * S, B[6] = (J * A - E * F - I * k) * S, B[7] = (H * F - C * A + V * k) * S, B[8] = (Q * g - Z * M + G * j) * S, B[9] = (X * M - W * g - $ * j) * S, B[10] = (E * P - U * A + I * R) * S, B[11] = (L * A - H * P - V * R) * S, B[12] = (Z * h - Q * p - O * j) * S, B[13] = (W * p - X * h + Y * j) * S, B[14] = (U * k - E * D - J * R) * S, B[15] = (H * D - L * k + C * R) * S, B;
};
var HB = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3], Q = K[4], Z = K[5], O = K[6], G = K[7], H = K[8], L = K[9], C = K[10], V = K[11], E = K[12], U = K[13], J = K[14], I = K[15];
  return B[0] = Z * (C * I - V * J) - L * (O * I - G * J) + U * (O * V - G * C), B[1] = -(X * (C * I - V * J) - L * (Y * I - $ * J) + U * (Y * V - $ * C)), B[2] = X * (O * I - G * J) - Z * (Y * I - $ * J) + U * (Y * G - $ * O), B[3] = -(X * (O * V - G * C) - Z * (Y * V - $ * C) + L * (Y * G - $ * O)), B[4] = -(Q * (C * I - V * J) - H * (O * I - G * J) + E * (O * V - G * C)), B[5] = W * (C * I - V * J) - H * (Y * I - $ * J) + E * (Y * V - $ * C), B[6] = -(W * (O * I - G * J) - Q * (Y * I - $ * J) + E * (Y * G - $ * O)), B[7] = W * (O * V - G * C) - Q * (Y * V - $ * C) + H * (Y * G - $ * O), B[8] = Q * (L * I - V * U) - H * (Z * I - G * U) + E * (Z * V - G * L), B[9] = -(W * (L * I - V * U) - H * (X * I - $ * U) + E * (X * V - $ * L)), B[10] = W * (Z * I - G * U) - Q * (X * I - $ * U) + E * (X * G - $ * Z), B[11] = -(W * (Z * V - G * L) - Q * (X * V - $ * L) + H * (X * G - $ * Z)), B[12] = -(Q * (L * J - C * U) - H * (Z * J - O * U) + E * (Z * C - O * L)), B[13] = W * (L * J - C * U) - H * (X * J - Y * U) + E * (X * C - Y * L), B[14] = -(W * (Z * J - O * U) - Q * (X * J - Y * U) + E * (X * O - Y * Z)), B[15] = W * (Z * C - O * L) - Q * (X * C - Y * L) + H * (X * O - Y * Z), B;
};
var JB = function(B) {
  var K = B[0], W = B[1], X = B[2], Y = B[3], $ = B[4], Q = B[5], Z = B[6], O = B[7], G = B[8], H = B[9], L = B[10], C = B[11], V = B[12], E = B[13], U = B[14], J = B[15], I = K * Q - W * $, R = K * Z - X * $, k = K * O - Y * $, A = W * Z - X * Q, D = W * O - Y * Q, P = X * O - Y * Z, F = G * E - H * V, j = G * U - L * V, h = G * J - C * V, M = H * U - L * E, p = H * J - C * E, g = L * J - C * U;
  return I * g - R * p + k * M + A * h - D * j + P * F;
};
var J0 = function(B, K, W) {
  var X = K[0], Y = K[1], $ = K[2], Q = K[3], Z = K[4], O = K[5], G = K[6], H = K[7], L = K[8], C = K[9], V = K[10], E = K[11], U = K[12], J = K[13], I = K[14], R = K[15], k = W[0], A = W[1], D = W[2], P = W[3];
  return B[0] = k * X + A * Z + D * L + P * U, B[1] = k * Y + A * O + D * C + P * J, B[2] = k * $ + A * G + D * V + P * I, B[3] = k * Q + A * H + D * E + P * R, k = W[4], A = W[5], D = W[6], P = W[7], B[4] = k * X + A * Z + D * L + P * U, B[5] = k * Y + A * O + D * C + P * J, B[6] = k * $ + A * G + D * V + P * I, B[7] = k * Q + A * H + D * E + P * R, k = W[8], A = W[9], D = W[10], P = W[11], B[8] = k * X + A * Z + D * L + P * U, B[9] = k * Y + A * O + D * C + P * J, B[10] = k * $ + A * G + D * V + P * I, B[11] = k * Q + A * H + D * E + P * R, k = W[12], A = W[13], D = W[14], P = W[15], B[12] = k * X + A * Z + D * L + P * U, B[13] = k * Y + A * O + D * C + P * J, B[14] = k * $ + A * G + D * V + P * I, B[15] = k * Q + A * H + D * E + P * R, B;
};
var LB = function(B, K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q, Z, O, G, H, L, C, V, E, U, J, I;
  if (K === B)
    B[12] = K[0] * X + K[4] * Y + K[8] * $ + K[12], B[13] = K[1] * X + K[5] * Y + K[9] * $ + K[13], B[14] = K[2] * X + K[6] * Y + K[10] * $ + K[14], B[15] = K[3] * X + K[7] * Y + K[11] * $ + K[15];
  else
    Q = K[0], Z = K[1], O = K[2], G = K[3], H = K[4], L = K[5], C = K[6], V = K[7], E = K[8], U = K[9], J = K[10], I = K[11], B[0] = Q, B[1] = Z, B[2] = O, B[3] = G, B[4] = H, B[5] = L, B[6] = C, B[7] = V, B[8] = E, B[9] = U, B[10] = J, B[11] = I, B[12] = Q * X + H * Y + E * $ + K[12], B[13] = Z * X + L * Y + U * $ + K[13], B[14] = O * X + C * Y + J * $ + K[14], B[15] = G * X + V * Y + I * $ + K[15];
  return B;
};
var VB = function(B, K, W) {
  var X = W[0], Y = W[1], $ = W[2];
  return B[0] = K[0] * X, B[1] = K[1] * X, B[2] = K[2] * X, B[3] = K[3] * X, B[4] = K[4] * Y, B[5] = K[5] * Y, B[6] = K[6] * Y, B[7] = K[7] * Y, B[8] = K[8] * $, B[9] = K[9] * $, B[10] = K[10] * $, B[11] = K[11] * $, B[12] = K[12], B[13] = K[13], B[14] = K[14], B[15] = K[15], B;
};
var CB = function(B, K, W, X) {
  var Y = X[0], $ = X[1], Q = X[2], Z = Math.hypot(Y, $, Q), O, G, H, L, C, V, E, U, J, I, R, k, A, D, P, F, j, h, M, p, g, q, S, f;
  if (Z < N)
    return null;
  if (Z = 1 / Z, Y *= Z, $ *= Z, Q *= Z, O = Math.sin(W), G = Math.cos(W), H = 1 - G, L = K[0], C = K[1], V = K[2], E = K[3], U = K[4], J = K[5], I = K[6], R = K[7], k = K[8], A = K[9], D = K[10], P = K[11], F = Y * Y * H + G, j = $ * Y * H + Q * O, h = Q * Y * H - $ * O, M = Y * $ * H - Q * O, p = $ * $ * H + G, g = Q * $ * H + Y * O, q = Y * Q * H + $ * O, S = $ * Q * H - Y * O, f = Q * Q * H + G, B[0] = L * F + U * j + k * h, B[1] = C * F + J * j + A * h, B[2] = V * F + I * j + D * h, B[3] = E * F + R * j + P * h, B[4] = L * M + U * p + k * g, B[5] = C * M + J * p + A * g, B[6] = V * M + I * p + D * g, B[7] = E * M + R * p + P * g, B[8] = L * q + U * S + k * f, B[9] = C * q + J * S + A * f, B[10] = V * q + I * S + D * f, B[11] = E * q + R * S + P * f, K !== B)
    B[12] = K[12], B[13] = K[13], B[14] = K[14], B[15] = K[15];
  return B;
};
var UB = function(B, K, W) {
  var X = Math.sin(W), Y = Math.cos(W), $ = K[4], Q = K[5], Z = K[6], O = K[7], G = K[8], H = K[9], L = K[10], C = K[11];
  if (K !== B)
    B[0] = K[0], B[1] = K[1], B[2] = K[2], B[3] = K[3], B[12] = K[12], B[13] = K[13], B[14] = K[14], B[15] = K[15];
  return B[4] = $ * Y + G * X, B[5] = Q * Y + H * X, B[6] = Z * Y + L * X, B[7] = O * Y + C * X, B[8] = G * Y - $ * X, B[9] = H * Y - Q * X, B[10] = L * Y - Z * X, B[11] = C * Y - O * X, B;
};
var EB = function(B, K, W) {
  var X = Math.sin(W), Y = Math.cos(W), $ = K[0], Q = K[1], Z = K[2], O = K[3], G = K[8], H = K[9], L = K[10], C = K[11];
  if (K !== B)
    B[4] = K[4], B[5] = K[5], B[6] = K[6], B[7] = K[7], B[12] = K[12], B[13] = K[13], B[14] = K[14], B[15] = K[15];
  return B[0] = $ * Y - G * X, B[1] = Q * Y - H * X, B[2] = Z * Y - L * X, B[3] = O * Y - C * X, B[8] = $ * X + G * Y, B[9] = Q * X + H * Y, B[10] = Z * X + L * Y, B[11] = O * X + C * Y, B;
};
var IB = function(B, K, W) {
  var X = Math.sin(W), Y = Math.cos(W), $ = K[0], Q = K[1], Z = K[2], O = K[3], G = K[4], H = K[5], L = K[6], C = K[7];
  if (K !== B)
    B[8] = K[8], B[9] = K[9], B[10] = K[10], B[11] = K[11], B[12] = K[12], B[13] = K[13], B[14] = K[14], B[15] = K[15];
  return B[0] = $ * Y + G * X, B[1] = Q * Y + H * X, B[2] = Z * Y + L * X, B[3] = O * Y + C * X, B[4] = G * Y - $ * X, B[5] = H * Y - Q * X, B[6] = L * Y - Z * X, B[7] = C * Y - O * X, B;
};
var DB = function(B, K) {
  return B[0] = 1, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = 1, B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[10] = 1, B[11] = 0, B[12] = K[0], B[13] = K[1], B[14] = K[2], B[15] = 1, B;
};
var PB = function(B, K) {
  return B[0] = K[0], B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = K[1], B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[10] = K[2], B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0, B[15] = 1, B;
};
var AB = function(B, K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q = Math.hypot(X, Y, $), Z, O, G;
  if (Q < N)
    return null;
  return Q = 1 / Q, X *= Q, Y *= Q, $ *= Q, Z = Math.sin(K), O = Math.cos(K), G = 1 - O, B[0] = X * X * G + O, B[1] = Y * X * G + $ * Z, B[2] = $ * X * G - Y * Z, B[3] = 0, B[4] = X * Y * G - $ * Z, B[5] = Y * Y * G + O, B[6] = $ * Y * G + X * Z, B[7] = 0, B[8] = X * $ * G + Y * Z, B[9] = Y * $ * G - X * Z, B[10] = $ * $ * G + O, B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0, B[15] = 1, B;
};
var kB = function(B, K) {
  var W = Math.sin(K), X = Math.cos(K);
  return B[0] = 1, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = X, B[6] = W, B[7] = 0, B[8] = 0, B[9] = -W, B[10] = X, B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0, B[15] = 1, B;
};
var NB = function(B, K) {
  var W = Math.sin(K), X = Math.cos(K);
  return B[0] = X, B[1] = 0, B[2] = -W, B[3] = 0, B[4] = 0, B[5] = 1, B[6] = 0, B[7] = 0, B[8] = W, B[9] = 0, B[10] = X, B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0, B[15] = 1, B;
};
var RB = function(B, K) {
  var W = Math.sin(K), X = Math.cos(K);
  return B[0] = X, B[1] = W, B[2] = 0, B[3] = 0, B[4] = -W, B[5] = X, B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[10] = 1, B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0, B[15] = 1, B;
};
var L0 = function(B, K, W) {
  var X = K[0], Y = K[1], $ = K[2], Q = K[3], Z = X + X, O = Y + Y, G = $ + $, H = X * Z, L = X * O, C = X * G, V = Y * O, E = Y * G, U = $ * G, J = Q * Z, I = Q * O, R = Q * G;
  return B[0] = 1 - (V + U), B[1] = L + R, B[2] = C - I, B[3] = 0, B[4] = L - R, B[5] = 1 - (H + U), B[6] = E + J, B[7] = 0, B[8] = C + I, B[9] = E - J, B[10] = 1 - (H + V), B[11] = 0, B[12] = W[0], B[13] = W[1], B[14] = W[2], B[15] = 1, B;
};
var SB = function(B, K) {
  var W = new _(3), X = -K[0], Y = -K[1], $ = -K[2], Q = K[3], Z = K[4], O = K[5], G = K[6], H = K[7], L = X * X + Y * Y + $ * $ + Q * Q;
  if (L > 0)
    W[0] = (Z * Q + H * X + O * $ - G * Y) * 2 / L, W[1] = (O * Q + H * Y + G * X - Z * $) * 2 / L, W[2] = (G * Q + H * $ + Z * Y - O * X) * 2 / L;
  else
    W[0] = (Z * Q + H * X + O * $ - G * Y) * 2, W[1] = (O * Q + H * Y + G * X - Z * $) * 2, W[2] = (G * Q + H * $ + Z * Y - O * X) * 2;
  return L0(B, K, W), B;
};
var TB = function(B, K) {
  return B[0] = K[12], B[1] = K[13], B[2] = K[14], B;
};
var V0 = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[4], Q = K[5], Z = K[6], O = K[8], G = K[9], H = K[10];
  return B[0] = Math.hypot(W, X, Y), B[1] = Math.hypot($, Q, Z), B[2] = Math.hypot(O, G, H), B;
};
var _B = function(B, K) {
  var W = new _(3);
  V0(W, K);
  var X = 1 / W[0], Y = 1 / W[1], $ = 1 / W[2], Q = K[0] * X, Z = K[1] * Y, O = K[2] * $, G = K[4] * X, H = K[5] * Y, L = K[6] * $, C = K[8] * X, V = K[9] * Y, E = K[10] * $, U = Q + H + E, J = 0;
  if (U > 0)
    J = Math.sqrt(U + 1) * 2, B[3] = 0.25 * J, B[0] = (L - V) / J, B[1] = (C - O) / J, B[2] = (Z - G) / J;
  else if (Q > H && Q > E)
    J = Math.sqrt(1 + Q - H - E) * 2, B[3] = (L - V) / J, B[0] = 0.25 * J, B[1] = (Z + G) / J, B[2] = (C + O) / J;
  else if (H > E)
    J = Math.sqrt(1 + H - Q - E) * 2, B[3] = (C - O) / J, B[0] = (Z + G) / J, B[1] = 0.25 * J, B[2] = (L + V) / J;
  else
    J = Math.sqrt(1 + E - Q - H) * 2, B[3] = (Z - G) / J, B[0] = (C + O) / J, B[1] = (L + V) / J, B[2] = 0.25 * J;
  return B;
};
var jB = function(B, K, W, X) {
  var Y = K[0], $ = K[1], Q = K[2], Z = K[3], O = Y + Y, G = $ + $, H = Q + Q, L = Y * O, C = Y * G, V = Y * H, E = $ * G, U = $ * H, J = Q * H, I = Z * O, R = Z * G, k = Z * H, A = X[0], D = X[1], P = X[2];
  return B[0] = (1 - (E + J)) * A, B[1] = (C + k) * A, B[2] = (V - R) * A, B[3] = 0, B[4] = (C - k) * D, B[5] = (1 - (L + J)) * D, B[6] = (U + I) * D, B[7] = 0, B[8] = (V + R) * P, B[9] = (U - I) * P, B[10] = (1 - (L + E)) * P, B[11] = 0, B[12] = W[0], B[13] = W[1], B[14] = W[2], B[15] = 1, B;
};
var hB = function(B, K, W, X, Y) {
  var $ = K[0], Q = K[1], Z = K[2], O = K[3], G = $ + $, H = Q + Q, L = Z + Z, C = $ * G, V = $ * H, E = $ * L, U = Q * H, J = Q * L, I = Z * L, R = O * G, k = O * H, A = O * L, D = X[0], P = X[1], F = X[2], j = Y[0], h = Y[1], M = Y[2], p = (1 - (U + I)) * D, g = (V + A) * D, q = (E - k) * D, S = (V - A) * P, f = (1 - (C + I)) * P, c = (J + R) * P, i = (E + k) * F, Z0 = (J - R) * F, O0 = (1 - (C + U)) * F;
  return B[0] = p, B[1] = g, B[2] = q, B[3] = 0, B[4] = S, B[5] = f, B[6] = c, B[7] = 0, B[8] = i, B[9] = Z0, B[10] = O0, B[11] = 0, B[12] = W[0] + j - (p * j + S * h + i * M), B[13] = W[1] + h - (g * j + f * h + Z0 * M), B[14] = W[2] + M - (q * j + c * h + O0 * M), B[15] = 1, B;
};
var MB = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3], Q = W + W, Z = X + X, O = Y + Y, G = W * Q, H = X * Q, L = X * Z, C = Y * Q, V = Y * Z, E = Y * O, U = $ * Q, J = $ * Z, I = $ * O;
  return B[0] = 1 - L - E, B[1] = H + I, B[2] = C - J, B[3] = 0, B[4] = H - I, B[5] = 1 - G - E, B[6] = V + U, B[7] = 0, B[8] = C + J, B[9] = V - U, B[10] = 1 - G - L, B[11] = 0, B[12] = 0, B[13] = 0, B[14] = 0, B[15] = 1, B;
};
var FB = function(B, K, W, X, Y, $, Q) {
  var Z = 1 / (W - K), O = 1 / (Y - X), G = 1 / ($ - Q);
  return B[0] = $ * 2 * Z, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = $ * 2 * O, B[6] = 0, B[7] = 0, B[8] = (W + K) * Z, B[9] = (Y + X) * O, B[10] = (Q + $) * G, B[11] = -1, B[12] = 0, B[13] = 0, B[14] = Q * $ * 2 * G, B[15] = 0, B;
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
  var Y = Math.tan(K.upDegrees * Math.PI / 180), $ = Math.tan(K.downDegrees * Math.PI / 180), Q = Math.tan(K.leftDegrees * Math.PI / 180), Z = Math.tan(K.rightDegrees * Math.PI / 180), O = 2 / (Q + Z), G = 2 / (Y + $);
  return B[0] = O, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = G, B[6] = 0, B[7] = 0, B[8] = -((Q - Z) * O * 0.5), B[9] = (Y - $) * G * 0.5, B[10] = X / (W - X), B[11] = -1, B[12] = 0, B[13] = 0, B[14] = X * W / (W - X), B[15] = 0, B;
};
var U0 = function(B, K, W, X, Y, $, Q) {
  var Z = 1 / (K - W), O = 1 / (X - Y), G = 1 / ($ - Q);
  return B[0] = -2 * Z, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = -2 * O, B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[10] = 2 * G, B[11] = 0, B[12] = (K + W) * Z, B[13] = (Y + X) * O, B[14] = (Q + $) * G, B[15] = 1, B;
};
var wB = function(B, K, W, X, Y, $, Q) {
  var Z = 1 / (K - W), O = 1 / (X - Y), G = 1 / ($ - Q);
  return B[0] = -2 * Z, B[1] = 0, B[2] = 0, B[3] = 0, B[4] = 0, B[5] = -2 * O, B[6] = 0, B[7] = 0, B[8] = 0, B[9] = 0, B[10] = G, B[11] = 0, B[12] = (K + W) * Z, B[13] = (Y + X) * O, B[14] = $ * G, B[15] = 1, B;
};
var dB = function(B, K, W, X) {
  var Y, $, Q, Z, O, G, H, L, C, V, E = K[0], U = K[1], J = K[2], I = X[0], R = X[1], k = X[2], A = W[0], D = W[1], P = W[2];
  if (Math.abs(E - A) < N && Math.abs(U - D) < N && Math.abs(J - P) < N)
    return H0(B);
  if (H = E - A, L = U - D, C = J - P, V = 1 / Math.hypot(H, L, C), H *= V, L *= V, C *= V, Y = R * C - k * L, $ = k * H - I * C, Q = I * L - R * H, V = Math.hypot(Y, $, Q), !V)
    Y = 0, $ = 0, Q = 0;
  else
    V = 1 / V, Y *= V, $ *= V, Q *= V;
  if (Z = L * Q - C * $, O = C * Y - H * Q, G = H * $ - L * Y, V = Math.hypot(Z, O, G), !V)
    Z = 0, O = 0, G = 0;
  else
    V = 1 / V, Z *= V, O *= V, G *= V;
  return B[0] = Y, B[1] = Z, B[2] = H, B[3] = 0, B[4] = $, B[5] = O, B[6] = L, B[7] = 0, B[8] = Q, B[9] = G, B[10] = C, B[11] = 0, B[12] = -(Y * E + $ * U + Q * J), B[13] = -(Z * E + O * U + G * J), B[14] = -(H * E + L * U + C * J), B[15] = 1, B;
};
var vB = function(B, K, W, X) {
  var Y = K[0], $ = K[1], Q = K[2], Z = X[0], O = X[1], G = X[2], H = Y - W[0], L = $ - W[1], C = Q - W[2], V = H * H + L * L + C * C;
  if (V > 0)
    V = 1 / Math.sqrt(V), H *= V, L *= V, C *= V;
  var E = O * C - G * L, U = G * H - Z * C, J = Z * L - O * H;
  if (V = E * E + U * U + J * J, V > 0)
    V = 1 / Math.sqrt(V), E *= V, U *= V, J *= V;
  return B[0] = E, B[1] = U, B[2] = J, B[3] = 0, B[4] = L * J - C * U, B[5] = C * E - H * J, B[6] = H * U - L * E, B[7] = 0, B[8] = H, B[9] = L, B[10] = C, B[11] = 0, B[12] = Y, B[13] = $, B[14] = Q, B[15] = 1, B;
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
var sB = function(B, K) {
  var W = B[0], X = B[1], Y = B[2], $ = B[3], Q = B[4], Z = B[5], O = B[6], G = B[7], H = B[8], L = B[9], C = B[10], V = B[11], E = B[12], U = B[13], J = B[14], I = B[15], R = K[0], k = K[1], A = K[2], D = K[3], P = K[4], F = K[5], j = K[6], h = K[7], M = K[8], p = K[9], g = K[10], q = K[11], S = K[12], f = K[13], c = K[14], i = K[15];
  return Math.abs(W - R) <= N * Math.max(1, Math.abs(W), Math.abs(R)) && Math.abs(X - k) <= N * Math.max(1, Math.abs(X), Math.abs(k)) && Math.abs(Y - A) <= N * Math.max(1, Math.abs(Y), Math.abs(A)) && Math.abs($ - D) <= N * Math.max(1, Math.abs($), Math.abs(D)) && Math.abs(Q - P) <= N * Math.max(1, Math.abs(Q), Math.abs(P)) && Math.abs(Z - F) <= N * Math.max(1, Math.abs(Z), Math.abs(F)) && Math.abs(O - j) <= N * Math.max(1, Math.abs(O), Math.abs(j)) && Math.abs(G - h) <= N * Math.max(1, Math.abs(G), Math.abs(h)) && Math.abs(H - M) <= N * Math.max(1, Math.abs(H), Math.abs(M)) && Math.abs(L - p) <= N * Math.max(1, Math.abs(L), Math.abs(p)) && Math.abs(C - g) <= N * Math.max(1, Math.abs(C), Math.abs(g)) && Math.abs(V - q) <= N * Math.max(1, Math.abs(V), Math.abs(q)) && Math.abs(E - S) <= N * Math.max(1, Math.abs(E), Math.abs(S)) && Math.abs(U - f) <= N * Math.max(1, Math.abs(U), Math.abs(f)) && Math.abs(J - c) <= N * Math.max(1, Math.abs(J), Math.abs(c)) && Math.abs(I - i) <= N * Math.max(1, Math.abs(I), Math.abs(i));
};
var x = function() {
  var B = new _(3);
  if (_ != Float32Array)
    B[0] = 0, B[1] = 0, B[2] = 0;
  return B;
};
var xB = function(B) {
  var K = new _(3);
  return K[0] = B[0], K[1] = B[1], K[2] = B[2], K;
};
var I0 = function(B) {
  var K = B[0], W = B[1], X = B[2];
  return Math.hypot(K, W, X);
};
var e = function(B, K, W) {
  var X = new _(3);
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
var R0 = function(B) {
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
  var X = K[0], Y = K[1], $ = K[2], Q = W[0], Z = W[1], O = W[2];
  return B[0] = Y * O - $ * Z, B[1] = $ * Q - X * O, B[2] = X * Z - Y * Q, B;
};
var QK = function(B, K, W, X) {
  var Y = K[0], $ = K[1], Q = K[2];
  return B[0] = Y + X * (W[0] - Y), B[1] = $ + X * (W[1] - $), B[2] = Q + X * (W[2] - Q), B;
};
var ZK = function(B, K, W, X, Y, $) {
  var Q = $ * $, Z = Q * (2 * $ - 3) + 1, O = Q * ($ - 2) + $, G = Q * ($ - 1), H = Q * (3 - 2 * $);
  return B[0] = K[0] * Z + W[0] * O + X[0] * G + Y[0] * H, B[1] = K[1] * Z + W[1] * O + X[1] * G + Y[1] * H, B[2] = K[2] * Z + W[2] * O + X[2] * G + Y[2] * H, B;
};
var OK = function(B, K, W, X, Y, $) {
  var Q = 1 - $, Z = Q * Q, O = $ * $, G = Z * Q, H = 3 * $ * Z, L = 3 * O * Q, C = O * $;
  return B[0] = K[0] * G + W[0] * H + X[0] * L + Y[0] * C, B[1] = K[1] * G + W[1] * H + X[1] * L + Y[1] * C, B[2] = K[2] * G + W[2] * H + X[2] * L + Y[2] * C, B;
};
var GK = function(B, K) {
  K = K || 1;
  var W = v() * 2 * Math.PI, X = v() * 2 - 1, Y = Math.sqrt(1 - X * X) * K;
  return B[0] = Math.cos(W) * Y, B[1] = Math.sin(W) * Y, B[2] = X * K, B;
};
var HK = function(B, K, W) {
  var X = K[0], Y = K[1], $ = K[2], Q = W[3] * X + W[7] * Y + W[11] * $ + W[15];
  return Q = Q || 1, B[0] = (W[0] * X + W[4] * Y + W[8] * $ + W[12]) / Q, B[1] = (W[1] * X + W[5] * Y + W[9] * $ + W[13]) / Q, B[2] = (W[2] * X + W[6] * Y + W[10] * $ + W[14]) / Q, B;
};
var JK = function(B, K, W) {
  var X = K[0], Y = K[1], $ = K[2];
  return B[0] = X * W[0] + Y * W[3] + $ * W[6], B[1] = X * W[1] + Y * W[4] + $ * W[7], B[2] = X * W[2] + Y * W[5] + $ * W[8], B;
};
var LK = function(B, K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q = W[3], Z = K[0], O = K[1], G = K[2], H = Y * G - $ * O, L = $ * Z - X * G, C = X * O - Y * Z, V = Y * C - $ * L, E = $ * H - X * C, U = X * L - Y * H, J = Q * 2;
  return H *= J, L *= J, C *= J, V *= 2, E *= 2, U *= 2, B[0] = Z + H + V, B[1] = O + L + E, B[2] = G + C + U, B;
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
  var W = B[0], X = B[1], Y = B[2], $ = K[0], Q = K[1], Z = K[2], O = Math.sqrt(W * W + X * X + Y * Y), G = Math.sqrt($ * $ + Q * Q + Z * Z), H = O * G, L = H && b(B, K) / H;
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
  var B = new _(4);
  if (_ != Float32Array)
    B[0] = 0, B[1] = 0, B[2] = 0, B[3] = 0;
  return B;
};
var S0 = function(B) {
  var K = new _(4);
  return K[0] = B[0], K[1] = B[1], K[2] = B[2], K[3] = B[3], K;
};
var T0 = function(B, K, W, X) {
  var Y = new _(4);
  return Y[0] = B, Y[1] = K, Y[2] = W, Y[3] = X, Y;
};
var _0 = function(B, K) {
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
var f0 = function(B, K, W, X) {
  var Y = K[0], $ = K[1], Q = K[2], Z = K[3];
  return B[0] = Y + X * (W[0] - Y), B[1] = $ + X * (W[1] - $), B[2] = Q + X * (W[2] - Q), B[3] = Z + X * (W[3] - Z), B;
};
var w0 = function(B, K) {
  return B[0] === K[0] && B[1] === K[1] && B[2] === K[2] && B[3] === K[3];
};
var d0 = function(B, K) {
  var W = B[0], X = B[1], Y = B[2], $ = B[3], Q = K[0], Z = K[1], O = K[2], G = K[3];
  return Math.abs(W - Q) <= N * Math.max(1, Math.abs(W), Math.abs(Q)) && Math.abs(X - Z) <= N * Math.max(1, Math.abs(X), Math.abs(Z)) && Math.abs(Y - O) <= N * Math.max(1, Math.abs(Y), Math.abs(O)) && Math.abs($ - G) <= N * Math.max(1, Math.abs($), Math.abs(G));
};
var W0 = function() {
  var B = new _(4);
  if (_ != Float32Array)
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
  var X = K[0], Y = K[1], $ = K[2], Q = K[3], Z = W[0], O = W[1], G = W[2], H = W[3];
  return B[0] = X * H + Q * Z + Y * G - $ * O, B[1] = Y * H + Q * O + $ * Z - X * G, B[2] = $ * H + Q * G + X * O - Y * Z, B[3] = Q * H - X * Z - Y * O - $ * G, B;
};
var qK = function(B, K, W) {
  W *= 0.5;
  var X = K[0], Y = K[1], $ = K[2], Q = K[3], Z = Math.sin(W), O = Math.cos(W);
  return B[0] = X * O + Q * Z, B[1] = Y * O + $ * Z, B[2] = $ * O - Y * Z, B[3] = Q * O - X * Z, B;
};
var fK = function(B, K, W) {
  W *= 0.5;
  var X = K[0], Y = K[1], $ = K[2], Q = K[3], Z = Math.sin(W), O = Math.cos(W);
  return B[0] = X * O - $ * Z, B[1] = Y * O + Q * Z, B[2] = $ * O + X * Z, B[3] = Q * O - Y * Z, B;
};
var wK = function(B, K, W) {
  W *= 0.5;
  var X = K[0], Y = K[1], $ = K[2], Q = K[3], Z = Math.sin(W), O = Math.cos(W);
  return B[0] = X * O + Y * Z, B[1] = Y * O - X * Z, B[2] = $ * O + Q * Z, B[3] = Q * O - $ * Z, B;
};
var dK = function(B, K) {
  var W = K[0], X = K[1], Y = K[2];
  return B[0] = W, B[1] = X, B[2] = Y, B[3] = Math.sqrt(Math.abs(1 - W * W - X * X - Y * Y)), B;
};
var l0 = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3], Q = Math.sqrt(W * W + X * X + Y * Y), Z = Math.exp($), O = Q > 0 ? Z * Math.sin(Q) / Q : 0;
  return B[0] = W * O, B[1] = X * O, B[2] = Y * O, B[3] = Z * Math.cos(Q), B;
};
var c0 = function(B, K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3], Q = Math.sqrt(W * W + X * X + Y * Y), Z = Q > 0 ? Math.atan2(Q, $) / Q : 0;
  return B[0] = W * Z, B[1] = X * Z, B[2] = Y * Z, B[3] = 0.5 * Math.log(W * W + X * X + Y * Y + $ * $), B;
};
var vK = function(B, K, W) {
  return c0(B, K), y0(B, B, W), l0(B, B), B;
};
var u = function(B, K, W, X) {
  var Y = K[0], $ = K[1], Q = K[2], Z = K[3], O = W[0], G = W[1], H = W[2], L = W[3], C, V, E, U, J;
  if (V = Y * O + $ * G + Q * H + Z * L, V < 0)
    V = -V, O = -O, G = -G, H = -H, L = -L;
  if (1 - V > N)
    C = Math.acos(V), E = Math.sin(C), U = Math.sin((1 - X) * C) / E, J = Math.sin(X * C) / E;
  else
    U = 1 - X, J = X;
  return B[0] = U * Y + J * O, B[1] = U * $ + J * G, B[2] = U * Q + J * H, B[3] = U * Z + J * L, B;
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
  var $ = Math.sin(K), Q = Math.cos(K), Z = Math.sin(W), O = Math.cos(W), G = Math.sin(X), H = Math.cos(X);
  return B[0] = $ * O * H - Q * Z * G, B[1] = Q * Z * H + $ * O * G, B[2] = Q * O * G - $ * Z * H, B[3] = Q * O * H + $ * Z * G, B;
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
var _ = typeof Float32Array !== "undefined" ? Float32Array : Array;
var v = Math.random;
var JW = Math.PI / 180;
if (!Math.hypot)
  Math.hypot = function() {
    var B = 0, K = arguments.length;
    while (K--)
      B += arguments[K] * arguments[K];
    return Math.sqrt(B);
  };
var T = {};
a(T, { transpose: () => {
  {
    return OB;
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
    return wB;
  }
}, orthoNO: () => {
  {
    return U0;
  }
}, ortho: () => {
  {
    return fB;
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
    return J0;
  }
}, mul: () => {
  {
    return rB;
  }
}, lookAt: () => {
  {
    return dB;
  }
}, invert: () => {
  {
    return GB;
  }
}, identity: () => {
  {
    return H0;
  }
}, getTranslation: () => {
  {
    return TB;
  }
}, getScaling: () => {
  {
    return V0;
  }
}, getRotation: () => {
  {
    return _B;
  }
}, frustum: () => {
  {
    return FB;
  }
}, fromZRotation: () => {
  {
    return RB;
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
    return SB;
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
    return sB;
  }
}, determinant: () => {
  {
    return JB;
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
    return HB;
  }
}, add: () => {
  {
    return cB;
  }
} });
var pB = C0;
var fB = U0;
var rB = J0;
var mB = E0;
var r = {};
a(r, { str: () => {
  {
    return yK;
  }
}, squaredLength: () => {
  {
    return r0;
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
    return wK;
  }
}, rotateY: () => {
  {
    return fK;
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
    return s0;
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
    return sK;
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
    return rK;
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
var s = {};
a(s, { zero: () => {
  {
    return IK;
  }
}, transformQuat: () => {
  {
    return LK;
  }
}, transformMat4: () => {
  {
    return HK;
  }
}, transformMat3: () => {
  {
    return JK;
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
    return R0;
  }
}, squaredDistance: () => {
  {
    return N0;
  }
}, sqrLen: () => {
  {
    return _K;
  }
}, sqrDist: () => {
  {
    return TK;
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
    return GK;
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
    return RK;
  }
}, distance: () => {
  {
    return k0;
  }
}, dist: () => {
  {
    return SK;
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
    return OK;
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
var RK = A0;
var SK = k0;
var TK = N0;
var K0 = I0;
var _K = R0;
var jK = function() {
  var B = x();
  return function(K, W, X, Y, $, Q) {
    var Z, O;
    if (!W)
      W = 3;
    if (!X)
      X = 0;
    if (Y)
      O = Math.min(Y * W + X, K.length);
    else
      O = K.length;
    for (Z = X;Z < O; Z += W)
      B[0] = K[Z], B[1] = K[Z + 1], B[2] = K[Z + 2], $(B, B, Q), K[Z] = B[0], K[Z + 1] = B[1], K[Z + 2] = B[2];
    return K;
  };
}();
var LW = function() {
  var B = hK();
  return function(K, W, X, Y, $, Q) {
    var Z, O;
    if (!W)
      W = 4;
    if (!X)
      X = 0;
    if (Y)
      O = Math.min(Y * W + X, K.length);
    else
      O = K.length;
    for (Z = X;Z < O; Z += W)
      B[0] = K[Z], B[1] = K[Z + 1], B[2] = K[Z + 2], B[3] = K[Z + 3], $(B, B, Q), K[Z] = B[0], K[Z + 1] = B[1], K[Z + 2] = B[2], K[Z + 3] = B[3];
    return K;
  };
}();
var zK = S0;
var sK = T0;
var rK = _0;
var mK = j0;
var xK = h0;
var eK = n0;
var y0 = M0;
var z0 = q0;
var bK = f0;
var s0 = F0;
var uK = s0;
var r0 = p0;
var oK = r0;
var X0 = g0;
var tK = w0;
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
  var B = G0();
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
var m0 = T.create();
var x0 = T.create();
var o = r.create();

class m {
  static HIDDEN = m.create().scale(0, 0, 0);
  static IDENTITY = m.create();
  #B = Float32Array.from(T.create());
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
    return T.copy(this.#B, B.getMatrix()), this.#K.onChange(), this;
  }
  identity() {
    return T.identity(this.#B), this.#K.onChange(), this;
  }
  invert(B) {
    return T.invert(this.#B, B?.getMatrix() ?? this.getMatrix()), this.#K.onChange(), this;
  }
  multiply(B) {
    return T.multiply(this.#B, this.#B, B.getMatrix()), this.#K.onChange(), this;
  }
  multiply2(B, K) {
    return T.multiply(this.#B, B.getMatrix(), K.getMatrix()), this.#K.onChange(), this;
  }
  multiply3(B, K, W) {
    return this.multiply2(B, K), this.multiply(W), this;
  }
  translate(B, K, W) {
    const X = Y0;
    return X[0] = B, X[1] = K, X[2] = W, this.move(X);
  }
  move(B) {
    return T.translate(this.#B, this.#B, B), this.#K.onChange(), this;
  }
  rotateX(B) {
    return T.rotateX(this.#B, this.#B, B), this.#K.onChange(), this;
  }
  rotateY(B) {
    return T.rotateY(this.#B, this.#B, B), this.#K.onChange(), this;
  }
  rotateZ(B) {
    return T.rotateZ(this.#B, this.#B, B), this.#K.onChange(), this;
  }
  setXRotation(B) {
    return T.fromXRotation(this.getMatrix(), B), this.#K.onChange(), this;
  }
  setYRotation(B) {
    return T.fromYRotation(this.getMatrix(), B), this.#K.onChange(), this;
  }
  scale(B, K, W) {
    return T.scale(this.#B, this.#B, [B, K ?? B, W ?? B]), this.#K.onChange(), this;
  }
  perspective(B, K, W, X) {
    return T.perspective(this.#B, B * XW, K, W, X), this.#K.onChange(), this;
  }
  ortho(B, K, W, X, Y, $) {
    return T.ortho(this.#B, B, K, W, X, Y, $), this.#K.onChange(), this;
  }
  combine(B, K, W = 0.5) {
    return T.multiplyScalar(m0, B.getMatrix(), 1 - W), T.multiplyScalar(x0, K.getMatrix(), W), T.add(this.#B, m0, x0), this.#K.onChange(), this;
  }
  static getMoveVector(B, K, W, X) {
    const Y = Y0;
    if (Y[0] = B, Y[1] = K, Y[2] = W, X)
      T.getRotation(o, X.getMatrix()), r.invert(o, o), s.transformQuat(Y, Y, o);
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
var w = m;

class e0 {
  w;
  q;
  #B;
  #K = false;
  #W = 0;
  #X;
  #Y;
  constructor(B, K, W) {
    this.getValue = K, this.apply = W, this.#Y = B, this.#B = this.getValue(B);
  }
  set element(B) {
    this.#Y = B, this.#B = this.getValue(B), this.#X = undefined;
  }
  setGoal(B, K, W) {
    if (this.#X && this.#X !== W)
      return;
    if (this.#B !== B || this.#W !== K)
      this.#W = K, this.#B = B, this.#X = W, this.#K = true;
  }
  get goal() {
    return this.#B;
  }
  update(B) {
    if (this.#K) {
      const K = this.getValue(this.#Y), W = this.goal - K, X = Math.min(Math.abs(W), this.#W * B);
      if (X <= 0.01)
        this.apply(this.#Y, this.goal), this.#K = false, this.#X = undefined;
      else
        this.apply(this.#Y, K + X * Math.sign(W));
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
  w;
  q;
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
var d;
(function(X) {
  X[X["AT_POSITION"] = 0] = "AT_POSITION";
  X[X["MOVED"] = 1] = "MOVED";
  X[X["BLOCKED"] = 2] = "BLOCKED";
})(d || (d = {}));
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
  #B = w.create().setPosition(0, 0, 0);
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
    const Y = w.getMoveVector(B, K, W, X), $ = $0(this.blockers, (Q) => Q.isBlocked(t(this.position[0] + Y[0], this.position[1] + Y[1], this.position[2] + Y[2], this.#K), this.position));
    if (!$)
      if (Y[0] || Y[1] || Y[2])
        this.#B.move(Y);
      else
        return d.AT_POSITION;
    return $ ? d.BLOCKED : d.MOVED;
  }
  moveTo(B, K, W) {
    if (this.position[0] === B && this.position[1] === K && this.position[2] === W)
      return d.AT_POSITION;
    const X = $0(this.blockers, (Y) => Y.isBlocked(t(B, K, W, this.#K), this.position));
    if (!X) {
      const [Y, $, Q] = this.#B.getPosition();
      if (Y !== B || $ !== K || Q !== W)
        this.#B.setPosition(B, K, W);
    }
    return X ? d.BLOCKED : d.MOVED;
  }
  movedTo(B, K, W) {
    return this.moveTo(B, K, W), this;
  }
  gotoPos(B, K, W, X = 0.1) {
    const Y = this.position, $ = B - Y[0], Q = K - Y[1], Z = W - Y[2], O = Math.sqrt($ * $ + Q * Q + Z * Z);
    if (O > 0.01) {
      const G = Math.min(O, X);
      return this.moveBy($ / O * G, Q / O * G, Z / O * G);
    } else
      return this.moveTo(B, K, W);
  }
  getMatrix() {
    return this.#B.getMatrix();
  }
}
var ZW = 1;
var OW = 1;

class a0 {
  #B = w.create();
  #K = w.create();
  #W = w.create();
  #X = [0, 0];
  #Y = new n(this);
  perspective;
  zoom;
  constructor() {
    const B = { onChange: () => {
      this.#B.combine(this.#W, this.#K, this.perspective.valueOf());
    } };
    this.perspective = new l(ZW, B.onChange), this.zoom = new l(OW, (K) => {
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
  w as Matrix
};
