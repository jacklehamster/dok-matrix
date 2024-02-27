// /Users/vincent/dok-matrix/example/node_modules/dok-matrix/dist/index.js
var O0 = function() {
  var K = new _(9);
  if (_ != Float32Array)
    K[1] = 0, K[2] = 0, K[3] = 0, K[5] = 0, K[6] = 0, K[7] = 0;
  return K[0] = 1, K[4] = 1, K[8] = 1, K;
};
var XK = function() {
  var K = new _(16);
  if (_ != Float32Array)
    K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0;
  return K[0] = 1, K[5] = 1, K[10] = 1, K[15] = 1, K;
};
var YK = function(K) {
  var W = new _(16);
  return W[0] = K[0], W[1] = K[1], W[2] = K[2], W[3] = K[3], W[4] = K[4], W[5] = K[5], W[6] = K[6], W[7] = K[7], W[8] = K[8], W[9] = K[9], W[10] = K[10], W[11] = K[11], W[12] = K[12], W[13] = K[13], W[14] = K[14], W[15] = K[15], W;
};
var $K = function(K, W) {
  return K[0] = W[0], K[1] = W[1], K[2] = W[2], K[3] = W[3], K[4] = W[4], K[5] = W[5], K[6] = W[6], K[7] = W[7], K[8] = W[8], K[9] = W[9], K[10] = W[10], K[11] = W[11], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15], K;
};
var BK = function(K, W, X, Y, $, B, Q, Z, O, G, H, L, C, V, D, E) {
  var J = new _(16);
  return J[0] = K, J[1] = W, J[2] = X, J[3] = Y, J[4] = $, J[5] = B, J[6] = Q, J[7] = Z, J[8] = O, J[9] = G, J[10] = H, J[11] = L, J[12] = C, J[13] = V, J[14] = D, J[15] = E, J;
};
var QK = function(K, W, X, Y, $, B, Q, Z, O, G, H, L, C, V, D, E, J) {
  return K[0] = W, K[1] = X, K[2] = Y, K[3] = $, K[4] = B, K[5] = Q, K[6] = Z, K[7] = O, K[8] = G, K[9] = H, K[10] = L, K[11] = C, K[12] = V, K[13] = D, K[14] = E, K[15] = J, K;
};
var G0 = function(K) {
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var ZK = function(K, W) {
  if (K === W) {
    var X = W[1], Y = W[2], $ = W[3], B = W[6], Q = W[7], Z = W[11];
    K[1] = W[4], K[2] = W[8], K[3] = W[12], K[4] = X, K[6] = W[9], K[7] = W[13], K[8] = Y, K[9] = B, K[11] = W[14], K[12] = $, K[13] = Q, K[14] = Z;
  } else
    K[0] = W[0], K[1] = W[4], K[2] = W[8], K[3] = W[12], K[4] = W[1], K[5] = W[5], K[6] = W[9], K[7] = W[13], K[8] = W[2], K[9] = W[6], K[10] = W[10], K[11] = W[14], K[12] = W[3], K[13] = W[7], K[14] = W[11], K[15] = W[15];
  return K;
};
var OK = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], B = W[3], Q = W[4], Z = W[5], O = W[6], G = W[7], H = W[8], L = W[9], C = W[10], V = W[11], D = W[12], E = W[13], J = W[14], I = W[15], N = X * Z - Y * Q, R = X * O - $ * Q, A = X * G - B * Q, U = Y * O - $ * Z, P = Y * G - B * Z, F = $ * G - B * O, j = H * E - L * D, h = H * J - C * D, M = H * I - V * D, p = L * J - C * E, g = L * I - V * E, q = C * I - V * J, k = N * q - R * g + A * p + U * M - P * h + F * j;
  if (!k)
    return null;
  return k = 1 / k, K[0] = (Z * q - O * g + G * p) * k, K[1] = ($ * g - Y * q - B * p) * k, K[2] = (E * F - J * P + I * U) * k, K[3] = (C * P - L * F - V * U) * k, K[4] = (O * M - Q * q - G * h) * k, K[5] = (X * q - $ * M + B * h) * k, K[6] = (J * A - D * F - I * R) * k, K[7] = (H * F - C * A + V * R) * k, K[8] = (Q * g - Z * M + G * j) * k, K[9] = (Y * M - X * g - B * j) * k, K[10] = (D * P - E * A + I * N) * k, K[11] = (L * A - H * P - V * N) * k, K[12] = (Z * h - Q * p - O * j) * k, K[13] = (X * p - Y * h + $ * j) * k, K[14] = (E * R - D * U - J * N) * k, K[15] = (H * U - L * R + C * N) * k, K;
};
var GK = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], B = W[3], Q = W[4], Z = W[5], O = W[6], G = W[7], H = W[8], L = W[9], C = W[10], V = W[11], D = W[12], E = W[13], J = W[14], I = W[15];
  return K[0] = Z * (C * I - V * J) - L * (O * I - G * J) + E * (O * V - G * C), K[1] = -(Y * (C * I - V * J) - L * ($ * I - B * J) + E * ($ * V - B * C)), K[2] = Y * (O * I - G * J) - Z * ($ * I - B * J) + E * ($ * G - B * O), K[3] = -(Y * (O * V - G * C) - Z * ($ * V - B * C) + L * ($ * G - B * O)), K[4] = -(Q * (C * I - V * J) - H * (O * I - G * J) + D * (O * V - G * C)), K[5] = X * (C * I - V * J) - H * ($ * I - B * J) + D * ($ * V - B * C), K[6] = -(X * (O * I - G * J) - Q * ($ * I - B * J) + D * ($ * G - B * O)), K[7] = X * (O * V - G * C) - Q * ($ * V - B * C) + H * ($ * G - B * O), K[8] = Q * (L * I - V * E) - H * (Z * I - G * E) + D * (Z * V - G * L), K[9] = -(X * (L * I - V * E) - H * (Y * I - B * E) + D * (Y * V - B * L)), K[10] = X * (Z * I - G * E) - Q * (Y * I - B * E) + D * (Y * G - B * Z), K[11] = -(X * (Z * V - G * L) - Q * (Y * V - B * L) + H * (Y * G - B * Z)), K[12] = -(Q * (L * J - C * E) - H * (Z * J - O * E) + D * (Z * C - O * L)), K[13] = X * (L * J - C * E) - H * (Y * J - $ * E) + D * (Y * C - $ * L), K[14] = -(X * (Z * J - O * E) - Q * (Y * J - $ * E) + D * (Y * O - $ * Z)), K[15] = X * (Z * C - O * L) - Q * (Y * C - $ * L) + H * (Y * O - $ * Z), K;
};
var HK = function(K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3], B = K[4], Q = K[5], Z = K[6], O = K[7], G = K[8], H = K[9], L = K[10], C = K[11], V = K[12], D = K[13], E = K[14], J = K[15], I = W * Q - X * B, N = W * Z - Y * B, R = W * O - $ * B, A = X * Z - Y * Q, U = X * O - $ * Q, P = Y * O - $ * Z, F = G * D - H * V, j = G * E - L * V, h = G * J - C * V, M = H * E - L * D, p = H * J - C * D, g = L * J - C * E;
  return I * g - N * p + R * M + A * h - U * j + P * F;
};
var H0 = function(K, W, X) {
  var Y = W[0], $ = W[1], B = W[2], Q = W[3], Z = W[4], O = W[5], G = W[6], H = W[7], L = W[8], C = W[9], V = W[10], D = W[11], E = W[12], J = W[13], I = W[14], N = W[15], R = X[0], A = X[1], U = X[2], P = X[3];
  return K[0] = R * Y + A * Z + U * L + P * E, K[1] = R * $ + A * O + U * C + P * J, K[2] = R * B + A * G + U * V + P * I, K[3] = R * Q + A * H + U * D + P * N, R = X[4], A = X[5], U = X[6], P = X[7], K[4] = R * Y + A * Z + U * L + P * E, K[5] = R * $ + A * O + U * C + P * J, K[6] = R * B + A * G + U * V + P * I, K[7] = R * Q + A * H + U * D + P * N, R = X[8], A = X[9], U = X[10], P = X[11], K[8] = R * Y + A * Z + U * L + P * E, K[9] = R * $ + A * O + U * C + P * J, K[10] = R * B + A * G + U * V + P * I, K[11] = R * Q + A * H + U * D + P * N, R = X[12], A = X[13], U = X[14], P = X[15], K[12] = R * Y + A * Z + U * L + P * E, K[13] = R * $ + A * O + U * C + P * J, K[14] = R * B + A * G + U * V + P * I, K[15] = R * Q + A * H + U * D + P * N, K;
};
var JK = function(K, W, X) {
  var Y = X[0], $ = X[1], B = X[2], Q, Z, O, G, H, L, C, V, D, E, J, I;
  if (W === K)
    K[12] = W[0] * Y + W[4] * $ + W[8] * B + W[12], K[13] = W[1] * Y + W[5] * $ + W[9] * B + W[13], K[14] = W[2] * Y + W[6] * $ + W[10] * B + W[14], K[15] = W[3] * Y + W[7] * $ + W[11] * B + W[15];
  else
    Q = W[0], Z = W[1], O = W[2], G = W[3], H = W[4], L = W[5], C = W[6], V = W[7], D = W[8], E = W[9], J = W[10], I = W[11], K[0] = Q, K[1] = Z, K[2] = O, K[3] = G, K[4] = H, K[5] = L, K[6] = C, K[7] = V, K[8] = D, K[9] = E, K[10] = J, K[11] = I, K[12] = Q * Y + H * $ + D * B + W[12], K[13] = Z * Y + L * $ + E * B + W[13], K[14] = O * Y + C * $ + J * B + W[14], K[15] = G * Y + V * $ + I * B + W[15];
  return K;
};
var LK = function(K, W, X) {
  var Y = X[0], $ = X[1], B = X[2];
  return K[0] = W[0] * Y, K[1] = W[1] * Y, K[2] = W[2] * Y, K[3] = W[3] * Y, K[4] = W[4] * $, K[5] = W[5] * $, K[6] = W[6] * $, K[7] = W[7] * $, K[8] = W[8] * B, K[9] = W[9] * B, K[10] = W[10] * B, K[11] = W[11] * B, K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15], K;
};
var VK = function(K, W, X, Y) {
  var $ = Y[0], B = Y[1], Q = Y[2], Z = Math.hypot($, B, Q), O, G, H, L, C, V, D, E, J, I, N, R, A, U, P, F, j, h, M, p, g, q, k, f;
  if (Z < T)
    return null;
  if (Z = 1 / Z, $ *= Z, B *= Z, Q *= Z, O = Math.sin(X), G = Math.cos(X), H = 1 - G, L = W[0], C = W[1], V = W[2], D = W[3], E = W[4], J = W[5], I = W[6], N = W[7], R = W[8], A = W[9], U = W[10], P = W[11], F = $ * $ * H + G, j = B * $ * H + Q * O, h = Q * $ * H - B * O, M = $ * B * H - Q * O, p = B * B * H + G, g = Q * B * H + $ * O, q = $ * Q * H + B * O, k = B * Q * H - $ * O, f = Q * Q * H + G, K[0] = L * F + E * j + R * h, K[1] = C * F + J * j + A * h, K[2] = V * F + I * j + U * h, K[3] = D * F + N * j + P * h, K[4] = L * M + E * p + R * g, K[5] = C * M + J * p + A * g, K[6] = V * M + I * p + U * g, K[7] = D * M + N * p + P * g, K[8] = L * q + E * k + R * f, K[9] = C * q + J * k + A * f, K[10] = V * q + I * k + U * f, K[11] = D * q + N * k + P * f, W !== K)
    K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K;
};
var CK = function(K, W, X) {
  var Y = Math.sin(X), $ = Math.cos(X), B = W[4], Q = W[5], Z = W[6], O = W[7], G = W[8], H = W[9], L = W[10], C = W[11];
  if (W !== K)
    K[0] = W[0], K[1] = W[1], K[2] = W[2], K[3] = W[3], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[4] = B * $ + G * Y, K[5] = Q * $ + H * Y, K[6] = Z * $ + L * Y, K[7] = O * $ + C * Y, K[8] = G * $ - B * Y, K[9] = H * $ - Q * Y, K[10] = L * $ - Z * Y, K[11] = C * $ - O * Y, K;
};
var EK = function(K, W, X) {
  var Y = Math.sin(X), $ = Math.cos(X), B = W[0], Q = W[1], Z = W[2], O = W[3], G = W[8], H = W[9], L = W[10], C = W[11];
  if (W !== K)
    K[4] = W[4], K[5] = W[5], K[6] = W[6], K[7] = W[7], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[0] = B * $ - G * Y, K[1] = Q * $ - H * Y, K[2] = Z * $ - L * Y, K[3] = O * $ - C * Y, K[8] = B * Y + G * $, K[9] = Q * Y + H * $, K[10] = Z * Y + L * $, K[11] = O * Y + C * $, K;
};
var DK = function(K, W, X) {
  var Y = Math.sin(X), $ = Math.cos(X), B = W[0], Q = W[1], Z = W[2], O = W[3], G = W[4], H = W[5], L = W[6], C = W[7];
  if (W !== K)
    K[8] = W[8], K[9] = W[9], K[10] = W[10], K[11] = W[11], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[0] = B * $ + G * Y, K[1] = Q * $ + H * Y, K[2] = Z * $ + L * Y, K[3] = O * $ + C * Y, K[4] = G * $ - B * Y, K[5] = H * $ - Q * Y, K[6] = L * $ - Z * Y, K[7] = C * $ - O * Y, K;
};
var IK = function(K, W) {
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = W[0], K[13] = W[1], K[14] = W[2], K[15] = 1, K;
};
var UK = function(K, W) {
  return K[0] = W[0], K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = W[1], K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = W[2], K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var PK = function(K, W, X) {
  var Y = X[0], $ = X[1], B = X[2], Q = Math.hypot(Y, $, B), Z, O, G;
  if (Q < T)
    return null;
  return Q = 1 / Q, Y *= Q, $ *= Q, B *= Q, Z = Math.sin(W), O = Math.cos(W), G = 1 - O, K[0] = Y * Y * G + O, K[1] = $ * Y * G + B * Z, K[2] = B * Y * G - $ * Z, K[3] = 0, K[4] = Y * $ * G - B * Z, K[5] = $ * $ * G + O, K[6] = B * $ * G + Y * Z, K[7] = 0, K[8] = Y * B * G + $ * Z, K[9] = $ * B * G - Y * Z, K[10] = B * B * G + O, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var AK = function(K, W) {
  var X = Math.sin(W), Y = Math.cos(W);
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = Y, K[6] = X, K[7] = 0, K[8] = 0, K[9] = -X, K[10] = Y, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var RK = function(K, W) {
  var X = Math.sin(W), Y = Math.cos(W);
  return K[0] = Y, K[1] = 0, K[2] = -X, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = X, K[9] = 0, K[10] = Y, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var TK = function(K, W) {
  var X = Math.sin(W), Y = Math.cos(W);
  return K[0] = Y, K[1] = X, K[2] = 0, K[3] = 0, K[4] = -X, K[5] = Y, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var J0 = function(K, W, X) {
  var Y = W[0], $ = W[1], B = W[2], Q = W[3], Z = Y + Y, O = $ + $, G = B + B, H = Y * Z, L = Y * O, C = Y * G, V = $ * O, D = $ * G, E = B * G, J = Q * Z, I = Q * O, N = Q * G;
  return K[0] = 1 - (V + E), K[1] = L + N, K[2] = C - I, K[3] = 0, K[4] = L - N, K[5] = 1 - (H + E), K[6] = D + J, K[7] = 0, K[8] = C + I, K[9] = D - J, K[10] = 1 - (H + V), K[11] = 0, K[12] = X[0], K[13] = X[1], K[14] = X[2], K[15] = 1, K;
};
var NK = function(K, W) {
  var X = new _(3), Y = -W[0], $ = -W[1], B = -W[2], Q = W[3], Z = W[4], O = W[5], G = W[6], H = W[7], L = Y * Y + $ * $ + B * B + Q * Q;
  if (L > 0)
    X[0] = (Z * Q + H * Y + O * B - G * $) * 2 / L, X[1] = (O * Q + H * $ + G * Y - Z * B) * 2 / L, X[2] = (G * Q + H * B + Z * $ - O * Y) * 2 / L;
  else
    X[0] = (Z * Q + H * Y + O * B - G * $) * 2, X[1] = (O * Q + H * $ + G * Y - Z * B) * 2, X[2] = (G * Q + H * B + Z * $ - O * Y) * 2;
  return J0(K, W, X), K;
};
var kK = function(K, W) {
  return K[0] = W[12], K[1] = W[13], K[2] = W[14], K;
};
var L0 = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], B = W[4], Q = W[5], Z = W[6], O = W[8], G = W[9], H = W[10];
  return K[0] = Math.hypot(X, Y, $), K[1] = Math.hypot(B, Q, Z), K[2] = Math.hypot(O, G, H), K;
};
var SK = function(K, W) {
  var X = new _(3);
  L0(X, W);
  var Y = 1 / X[0], $ = 1 / X[1], B = 1 / X[2], Q = W[0] * Y, Z = W[1] * $, O = W[2] * B, G = W[4] * Y, H = W[5] * $, L = W[6] * B, C = W[8] * Y, V = W[9] * $, D = W[10] * B, E = Q + H + D, J = 0;
  if (E > 0)
    J = Math.sqrt(E + 1) * 2, K[3] = 0.25 * J, K[0] = (L - V) / J, K[1] = (C - O) / J, K[2] = (Z - G) / J;
  else if (Q > H && Q > D)
    J = Math.sqrt(1 + Q - H - D) * 2, K[3] = (L - V) / J, K[0] = 0.25 * J, K[1] = (Z + G) / J, K[2] = (C + O) / J;
  else if (H > D)
    J = Math.sqrt(1 + H - Q - D) * 2, K[3] = (C - O) / J, K[0] = (Z + G) / J, K[1] = 0.25 * J, K[2] = (L + V) / J;
  else
    J = Math.sqrt(1 + D - Q - H) * 2, K[3] = (Z - G) / J, K[0] = (C + O) / J, K[1] = (L + V) / J, K[2] = 0.25 * J;
  return K;
};
var _K = function(K, W, X, Y) {
  var $ = W[0], B = W[1], Q = W[2], Z = W[3], O = $ + $, G = B + B, H = Q + Q, L = $ * O, C = $ * G, V = $ * H, D = B * G, E = B * H, J = Q * H, I = Z * O, N = Z * G, R = Z * H, A = Y[0], U = Y[1], P = Y[2];
  return K[0] = (1 - (D + J)) * A, K[1] = (C + R) * A, K[2] = (V - N) * A, K[3] = 0, K[4] = (C - R) * U, K[5] = (1 - (L + J)) * U, K[6] = (E + I) * U, K[7] = 0, K[8] = (V + N) * P, K[9] = (E - I) * P, K[10] = (1 - (L + D)) * P, K[11] = 0, K[12] = X[0], K[13] = X[1], K[14] = X[2], K[15] = 1, K;
};
var jK = function(K, W, X, Y, $) {
  var B = W[0], Q = W[1], Z = W[2], O = W[3], G = B + B, H = Q + Q, L = Z + Z, C = B * G, V = B * H, D = B * L, E = Q * H, J = Q * L, I = Z * L, N = O * G, R = O * H, A = O * L, U = Y[0], P = Y[1], F = Y[2], j = $[0], h = $[1], M = $[2], p = (1 - (E + I)) * U, g = (V + A) * U, q = (D - R) * U, k = (V - A) * P, f = (1 - (C + I)) * P, i = (J + N) * P, y = (D + R) * F, Q0 = (J - N) * F, Z0 = (1 - (C + E)) * F;
  return K[0] = p, K[1] = g, K[2] = q, K[3] = 0, K[4] = k, K[5] = f, K[6] = i, K[7] = 0, K[8] = y, K[9] = Q0, K[10] = Z0, K[11] = 0, K[12] = X[0] + j - (p * j + k * h + y * M), K[13] = X[1] + h - (g * j + f * h + Q0 * M), K[14] = X[2] + M - (q * j + i * h + Z0 * M), K[15] = 1, K;
};
var hK = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], B = W[3], Q = X + X, Z = Y + Y, O = $ + $, G = X * Q, H = Y * Q, L = Y * Z, C = $ * Q, V = $ * Z, D = $ * O, E = B * Q, J = B * Z, I = B * O;
  return K[0] = 1 - L - D, K[1] = H + I, K[2] = C - J, K[3] = 0, K[4] = H - I, K[5] = 1 - G - D, K[6] = V + E, K[7] = 0, K[8] = C + J, K[9] = V - E, K[10] = 1 - G - L, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var MK = function(K, W, X, Y, $, B, Q) {
  var Z = 1 / (X - W), O = 1 / ($ - Y), G = 1 / (B - Q);
  return K[0] = B * 2 * Z, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = B * 2 * O, K[6] = 0, K[7] = 0, K[8] = (X + W) * Z, K[9] = ($ + Y) * O, K[10] = (Q + B) * G, K[11] = -1, K[12] = 0, K[13] = 0, K[14] = Q * B * 2 * G, K[15] = 0, K;
};
var V0 = function(K, W, X, Y, $) {
  var B = 1 / Math.tan(W / 2), Q;
  if (K[0] = B / X, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = B, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = -1, K[12] = 0, K[13] = 0, K[15] = 0, $ != null && $ !== Infinity)
    Q = 1 / (Y - $), K[10] = ($ + Y) * Q, K[14] = 2 * $ * Y * Q;
  else
    K[10] = -1, K[14] = -2 * Y;
  return K;
};
var pK = function(K, W, X, Y, $) {
  var B = 1 / Math.tan(W / 2), Q;
  if (K[0] = B / X, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = B, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = -1, K[12] = 0, K[13] = 0, K[15] = 0, $ != null && $ !== Infinity)
    Q = 1 / (Y - $), K[10] = $ * Q, K[14] = $ * Y * Q;
  else
    K[10] = -1, K[14] = -Y;
  return K;
};
var gK = function(K, W, X, Y) {
  var $ = Math.tan(W.upDegrees * Math.PI / 180), B = Math.tan(W.downDegrees * Math.PI / 180), Q = Math.tan(W.leftDegrees * Math.PI / 180), Z = Math.tan(W.rightDegrees * Math.PI / 180), O = 2 / (Q + Z), G = 2 / ($ + B);
  return K[0] = O, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = G, K[6] = 0, K[7] = 0, K[8] = -((Q - Z) * O * 0.5), K[9] = ($ - B) * G * 0.5, K[10] = Y / (X - Y), K[11] = -1, K[12] = 0, K[13] = 0, K[14] = Y * X / (X - Y), K[15] = 0, K;
};
var C0 = function(K, W, X, Y, $, B, Q) {
  var Z = 1 / (W - X), O = 1 / (Y - $), G = 1 / (B - Q);
  return K[0] = -2 * Z, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = -2 * O, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 2 * G, K[11] = 0, K[12] = (W + X) * Z, K[13] = ($ + Y) * O, K[14] = (Q + B) * G, K[15] = 1, K;
};
var fK = function(K, W, X, Y, $, B, Q) {
  var Z = 1 / (W - X), O = 1 / (Y - $), G = 1 / (B - Q);
  return K[0] = -2 * Z, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = -2 * O, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = G, K[11] = 0, K[12] = (W + X) * Z, K[13] = ($ + Y) * O, K[14] = B * G, K[15] = 1, K;
};
var wK = function(K, W, X, Y) {
  var $, B, Q, Z, O, G, H, L, C, V, D = W[0], E = W[1], J = W[2], I = Y[0], N = Y[1], R = Y[2], A = X[0], U = X[1], P = X[2];
  if (Math.abs(D - A) < T && Math.abs(E - U) < T && Math.abs(J - P) < T)
    return G0(K);
  if (H = D - A, L = E - U, C = J - P, V = 1 / Math.hypot(H, L, C), H *= V, L *= V, C *= V, $ = N * C - R * L, B = R * H - I * C, Q = I * L - N * H, V = Math.hypot($, B, Q), !V)
    $ = 0, B = 0, Q = 0;
  else
    V = 1 / V, $ *= V, B *= V, Q *= V;
  if (Z = L * Q - C * B, O = C * $ - H * Q, G = H * B - L * $, V = Math.hypot(Z, O, G), !V)
    Z = 0, O = 0, G = 0;
  else
    V = 1 / V, Z *= V, O *= V, G *= V;
  return K[0] = $, K[1] = Z, K[2] = H, K[3] = 0, K[4] = B, K[5] = O, K[6] = L, K[7] = 0, K[8] = Q, K[9] = G, K[10] = C, K[11] = 0, K[12] = -($ * D + B * E + Q * J), K[13] = -(Z * D + O * E + G * J), K[14] = -(H * D + L * E + C * J), K[15] = 1, K;
};
var dK = function(K, W, X, Y) {
  var $ = W[0], B = W[1], Q = W[2], Z = Y[0], O = Y[1], G = Y[2], H = $ - X[0], L = B - X[1], C = Q - X[2], V = H * H + L * L + C * C;
  if (V > 0)
    V = 1 / Math.sqrt(V), H *= V, L *= V, C *= V;
  var D = O * C - G * L, E = G * H - Z * C, J = Z * L - O * H;
  if (V = D * D + E * E + J * J, V > 0)
    V = 1 / Math.sqrt(V), D *= V, E *= V, J *= V;
  return K[0] = D, K[1] = E, K[2] = J, K[3] = 0, K[4] = L * J - C * E, K[5] = C * D - H * J, K[6] = H * E - L * D, K[7] = 0, K[8] = H, K[9] = L, K[10] = C, K[11] = 0, K[12] = $, K[13] = B, K[14] = Q, K[15] = 1, K;
};
var vK = function(K) {
  return "mat4(" + K[0] + ", " + K[1] + ", " + K[2] + ", " + K[3] + ", " + K[4] + ", " + K[5] + ", " + K[6] + ", " + K[7] + ", " + K[8] + ", " + K[9] + ", " + K[10] + ", " + K[11] + ", " + K[12] + ", " + K[13] + ", " + K[14] + ", " + K[15] + ")";
};
var nK = function(K) {
  return Math.hypot(K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8], K[9], K[10], K[11], K[12], K[13], K[14], K[15]);
};
var lK = function(K, W, X) {
  return K[0] = W[0] + X[0], K[1] = W[1] + X[1], K[2] = W[2] + X[2], K[3] = W[3] + X[3], K[4] = W[4] + X[4], K[5] = W[5] + X[5], K[6] = W[6] + X[6], K[7] = W[7] + X[7], K[8] = W[8] + X[8], K[9] = W[9] + X[9], K[10] = W[10] + X[10], K[11] = W[11] + X[11], K[12] = W[12] + X[12], K[13] = W[13] + X[13], K[14] = W[14] + X[14], K[15] = W[15] + X[15], K;
};
var E0 = function(K, W, X) {
  return K[0] = W[0] - X[0], K[1] = W[1] - X[1], K[2] = W[2] - X[2], K[3] = W[3] - X[3], K[4] = W[4] - X[4], K[5] = W[5] - X[5], K[6] = W[6] - X[6], K[7] = W[7] - X[7], K[8] = W[8] - X[8], K[9] = W[9] - X[9], K[10] = W[10] - X[10], K[11] = W[11] - X[11], K[12] = W[12] - X[12], K[13] = W[13] - X[13], K[14] = W[14] - X[14], K[15] = W[15] - X[15], K;
};
var cK = function(K, W, X) {
  return K[0] = W[0] * X, K[1] = W[1] * X, K[2] = W[2] * X, K[3] = W[3] * X, K[4] = W[4] * X, K[5] = W[5] * X, K[6] = W[6] * X, K[7] = W[7] * X, K[8] = W[8] * X, K[9] = W[9] * X, K[10] = W[10] * X, K[11] = W[11] * X, K[12] = W[12] * X, K[13] = W[13] * X, K[14] = W[14] * X, K[15] = W[15] * X, K;
};
var iK = function(K, W, X, Y) {
  return K[0] = W[0] + X[0] * Y, K[1] = W[1] + X[1] * Y, K[2] = W[2] + X[2] * Y, K[3] = W[3] + X[3] * Y, K[4] = W[4] + X[4] * Y, K[5] = W[5] + X[5] * Y, K[6] = W[6] + X[6] * Y, K[7] = W[7] + X[7] * Y, K[8] = W[8] + X[8] * Y, K[9] = W[9] + X[9] * Y, K[10] = W[10] + X[10] * Y, K[11] = W[11] + X[11] * Y, K[12] = W[12] + X[12] * Y, K[13] = W[13] + X[13] * Y, K[14] = W[14] + X[14] * Y, K[15] = W[15] + X[15] * Y, K;
};
var yK = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2] && K[3] === W[3] && K[4] === W[4] && K[5] === W[5] && K[6] === W[6] && K[7] === W[7] && K[8] === W[8] && K[9] === W[9] && K[10] === W[10] && K[11] === W[11] && K[12] === W[12] && K[13] === W[13] && K[14] === W[14] && K[15] === W[15];
};
var zK = function(K, W) {
  var X = K[0], Y = K[1], $ = K[2], B = K[3], Q = K[4], Z = K[5], O = K[6], G = K[7], H = K[8], L = K[9], C = K[10], V = K[11], D = K[12], E = K[13], J = K[14], I = K[15], N = W[0], R = W[1], A = W[2], U = W[3], P = W[4], F = W[5], j = W[6], h = W[7], M = W[8], p = W[9], g = W[10], q = W[11], k = W[12], f = W[13], i = W[14], y = W[15];
  return Math.abs(X - N) <= T * Math.max(1, Math.abs(X), Math.abs(N)) && Math.abs(Y - R) <= T * Math.max(1, Math.abs(Y), Math.abs(R)) && Math.abs($ - A) <= T * Math.max(1, Math.abs($), Math.abs(A)) && Math.abs(B - U) <= T * Math.max(1, Math.abs(B), Math.abs(U)) && Math.abs(Q - P) <= T * Math.max(1, Math.abs(Q), Math.abs(P)) && Math.abs(Z - F) <= T * Math.max(1, Math.abs(Z), Math.abs(F)) && Math.abs(O - j) <= T * Math.max(1, Math.abs(O), Math.abs(j)) && Math.abs(G - h) <= T * Math.max(1, Math.abs(G), Math.abs(h)) && Math.abs(H - M) <= T * Math.max(1, Math.abs(H), Math.abs(M)) && Math.abs(L - p) <= T * Math.max(1, Math.abs(L), Math.abs(p)) && Math.abs(C - g) <= T * Math.max(1, Math.abs(C), Math.abs(g)) && Math.abs(V - q) <= T * Math.max(1, Math.abs(V), Math.abs(q)) && Math.abs(D - k) <= T * Math.max(1, Math.abs(D), Math.abs(k)) && Math.abs(E - f) <= T * Math.max(1, Math.abs(E), Math.abs(f)) && Math.abs(J - i) <= T * Math.max(1, Math.abs(J), Math.abs(i)) && Math.abs(I - y) <= T * Math.max(1, Math.abs(I), Math.abs(y));
};
var e = function() {
  var K = new _(3);
  if (_ != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0;
  return K;
};
var mK = function(K) {
  var W = new _(3);
  return W[0] = K[0], W[1] = K[1], W[2] = K[2], W;
};
var D0 = function(K) {
  var W = K[0], X = K[1], Y = K[2];
  return Math.hypot(W, X, Y);
};
var b = function(K, W, X) {
  var Y = new _(3);
  return Y[0] = K, Y[1] = W, Y[2] = X, Y;
};
var xK = function(K, W) {
  return K[0] = W[0], K[1] = W[1], K[2] = W[2], K;
};
var eK = function(K, W, X, Y) {
  return K[0] = W, K[1] = X, K[2] = Y, K;
};
var bK = function(K, W, X) {
  return K[0] = W[0] + X[0], K[1] = W[1] + X[1], K[2] = W[2] + X[2], K;
};
var I0 = function(K, W, X) {
  return K[0] = W[0] - X[0], K[1] = W[1] - X[1], K[2] = W[2] - X[2], K;
};
var U0 = function(K, W, X) {
  return K[0] = W[0] * X[0], K[1] = W[1] * X[1], K[2] = W[2] * X[2], K;
};
var P0 = function(K, W, X) {
  return K[0] = W[0] / X[0], K[1] = W[1] / X[1], K[2] = W[2] / X[2], K;
};
var uK = function(K, W) {
  return K[0] = Math.ceil(W[0]), K[1] = Math.ceil(W[1]), K[2] = Math.ceil(W[2]), K;
};
var oK = function(K, W) {
  return K[0] = Math.floor(W[0]), K[1] = Math.floor(W[1]), K[2] = Math.floor(W[2]), K;
};
var tK = function(K, W, X) {
  return K[0] = Math.min(W[0], X[0]), K[1] = Math.min(W[1], X[1]), K[2] = Math.min(W[2], X[2]), K;
};
var aK = function(K, W, X) {
  return K[0] = Math.max(W[0], X[0]), K[1] = Math.max(W[1], X[1]), K[2] = Math.max(W[2], X[2]), K;
};
var KW = function(K, W) {
  return K[0] = Math.round(W[0]), K[1] = Math.round(W[1]), K[2] = Math.round(W[2]), K;
};
var WW = function(K, W, X) {
  return K[0] = W[0] * X, K[1] = W[1] * X, K[2] = W[2] * X, K;
};
var XW = function(K, W, X, Y) {
  return K[0] = W[0] + X[0] * Y, K[1] = W[1] + X[1] * Y, K[2] = W[2] + X[2] * Y, K;
};
var A0 = function(K, W) {
  var X = W[0] - K[0], Y = W[1] - K[1], $ = W[2] - K[2];
  return Math.hypot(X, Y, $);
};
var R0 = function(K, W) {
  var X = W[0] - K[0], Y = W[1] - K[1], $ = W[2] - K[2];
  return X * X + Y * Y + $ * $;
};
var T0 = function(K) {
  var W = K[0], X = K[1], Y = K[2];
  return W * W + X * X + Y * Y;
};
var YW = function(K, W) {
  return K[0] = -W[0], K[1] = -W[1], K[2] = -W[2], K;
};
var $W = function(K, W) {
  return K[0] = 1 / W[0], K[1] = 1 / W[1], K[2] = 1 / W[2], K;
};
var K0 = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], B = X * X + Y * Y + $ * $;
  if (B > 0)
    B = 1 / Math.sqrt(B);
  return K[0] = W[0] * B, K[1] = W[1] * B, K[2] = W[2] * B, K;
};
var u = function(K, W) {
  return K[0] * W[0] + K[1] * W[1] + K[2] * W[2];
};
var s = function(K, W, X) {
  var Y = W[0], $ = W[1], B = W[2], Q = X[0], Z = X[1], O = X[2];
  return K[0] = $ * O - B * Z, K[1] = B * Q - Y * O, K[2] = Y * Z - $ * Q, K;
};
var BW = function(K, W, X, Y) {
  var $ = W[0], B = W[1], Q = W[2];
  return K[0] = $ + Y * (X[0] - $), K[1] = B + Y * (X[1] - B), K[2] = Q + Y * (X[2] - Q), K;
};
var QW = function(K, W, X, Y, $, B) {
  var Q = B * B, Z = Q * (2 * B - 3) + 1, O = Q * (B - 2) + B, G = Q * (B - 1), H = Q * (3 - 2 * B);
  return K[0] = W[0] * Z + X[0] * O + Y[0] * G + $[0] * H, K[1] = W[1] * Z + X[1] * O + Y[1] * G + $[1] * H, K[2] = W[2] * Z + X[2] * O + Y[2] * G + $[2] * H, K;
};
var ZW = function(K, W, X, Y, $, B) {
  var Q = 1 - B, Z = Q * Q, O = B * B, G = Z * Q, H = 3 * B * Z, L = 3 * O * Q, C = O * B;
  return K[0] = W[0] * G + X[0] * H + Y[0] * L + $[0] * C, K[1] = W[1] * G + X[1] * H + Y[1] * L + $[1] * C, K[2] = W[2] * G + X[2] * H + Y[2] * L + $[2] * C, K;
};
var OW = function(K, W) {
  W = W || 1;
  var X = v() * 2 * Math.PI, Y = v() * 2 - 1, $ = Math.sqrt(1 - Y * Y) * W;
  return K[0] = Math.cos(X) * $, K[1] = Math.sin(X) * $, K[2] = Y * W, K;
};
var GW = function(K, W, X) {
  var Y = W[0], $ = W[1], B = W[2], Q = X[3] * Y + X[7] * $ + X[11] * B + X[15];
  return Q = Q || 1, K[0] = (X[0] * Y + X[4] * $ + X[8] * B + X[12]) / Q, K[1] = (X[1] * Y + X[5] * $ + X[9] * B + X[13]) / Q, K[2] = (X[2] * Y + X[6] * $ + X[10] * B + X[14]) / Q, K;
};
var HW = function(K, W, X) {
  var Y = W[0], $ = W[1], B = W[2];
  return K[0] = Y * X[0] + $ * X[3] + B * X[6], K[1] = Y * X[1] + $ * X[4] + B * X[7], K[2] = Y * X[2] + $ * X[5] + B * X[8], K;
};
var JW = function(K, W, X) {
  var Y = X[0], $ = X[1], B = X[2], Q = X[3], Z = W[0], O = W[1], G = W[2], H = $ * G - B * O, L = B * Z - Y * G, C = Y * O - $ * Z, V = $ * C - B * L, D = B * H - Y * C, E = Y * L - $ * H, J = Q * 2;
  return H *= J, L *= J, C *= J, V *= 2, D *= 2, E *= 2, K[0] = Z + H + V, K[1] = O + L + D, K[2] = G + C + E, K;
};
var LW = function(K, W, X, Y) {
  var $ = [], B = [];
  return $[0] = W[0] - X[0], $[1] = W[1] - X[1], $[2] = W[2] - X[2], B[0] = $[0], B[1] = $[1] * Math.cos(Y) - $[2] * Math.sin(Y), B[2] = $[1] * Math.sin(Y) + $[2] * Math.cos(Y), K[0] = B[0] + X[0], K[1] = B[1] + X[1], K[2] = B[2] + X[2], K;
};
var VW = function(K, W, X, Y) {
  var $ = [], B = [];
  return $[0] = W[0] - X[0], $[1] = W[1] - X[1], $[2] = W[2] - X[2], B[0] = $[2] * Math.sin(Y) + $[0] * Math.cos(Y), B[1] = $[1], B[2] = $[2] * Math.cos(Y) - $[0] * Math.sin(Y), K[0] = B[0] + X[0], K[1] = B[1] + X[1], K[2] = B[2] + X[2], K;
};
var CW = function(K, W, X, Y) {
  var $ = [], B = [];
  return $[0] = W[0] - X[0], $[1] = W[1] - X[1], $[2] = W[2] - X[2], B[0] = $[0] * Math.cos(Y) - $[1] * Math.sin(Y), B[1] = $[0] * Math.sin(Y) + $[1] * Math.cos(Y), B[2] = $[2], K[0] = B[0] + X[0], K[1] = B[1] + X[1], K[2] = B[2] + X[2], K;
};
var EW = function(K, W) {
  var X = K[0], Y = K[1], $ = K[2], B = W[0], Q = W[1], Z = W[2], O = Math.sqrt(X * X + Y * Y + $ * $), G = Math.sqrt(B * B + Q * Q + Z * Z), H = O * G, L = H && u(K, W) / H;
  return Math.acos(Math.min(Math.max(L, -1), 1));
};
var DW = function(K) {
  return K[0] = 0, K[1] = 0, K[2] = 0, K;
};
var IW = function(K) {
  return "vec3(" + K[0] + ", " + K[1] + ", " + K[2] + ")";
};
var UW = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2];
};
var PW = function(K, W) {
  var X = K[0], Y = K[1], $ = K[2], B = W[0], Q = W[1], Z = W[2];
  return Math.abs(X - B) <= T * Math.max(1, Math.abs(X), Math.abs(B)) && Math.abs(Y - Q) <= T * Math.max(1, Math.abs(Y), Math.abs(Q)) && Math.abs($ - Z) <= T * Math.max(1, Math.abs($), Math.abs(Z));
};
var jW = function() {
  var K = new _(4);
  if (_ != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0, K[3] = 0;
  return K;
};
var N0 = function(K) {
  var W = new _(4);
  return W[0] = K[0], W[1] = K[1], W[2] = K[2], W[3] = K[3], W;
};
var k0 = function(K, W, X, Y) {
  var $ = new _(4);
  return $[0] = K, $[1] = W, $[2] = X, $[3] = Y, $;
};
var S0 = function(K, W) {
  return K[0] = W[0], K[1] = W[1], K[2] = W[2], K[3] = W[3], K;
};
var _0 = function(K, W, X, Y, $) {
  return K[0] = W, K[1] = X, K[2] = Y, K[3] = $, K;
};
var j0 = function(K, W, X) {
  return K[0] = W[0] + X[0], K[1] = W[1] + X[1], K[2] = W[2] + X[2], K[3] = W[3] + X[3], K;
};
var h0 = function(K, W, X) {
  return K[0] = W[0] * X, K[1] = W[1] * X, K[2] = W[2] * X, K[3] = W[3] * X, K;
};
var M0 = function(K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3];
  return Math.hypot(W, X, Y, $);
};
var F0 = function(K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3];
  return W * W + X * X + Y * Y + $ * $;
};
var p0 = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], B = W[3], Q = X * X + Y * Y + $ * $ + B * B;
  if (Q > 0)
    Q = 1 / Math.sqrt(Q);
  return K[0] = X * Q, K[1] = Y * Q, K[2] = $ * Q, K[3] = B * Q, K;
};
var g0 = function(K, W) {
  return K[0] * W[0] + K[1] * W[1] + K[2] * W[2] + K[3] * W[3];
};
var q0 = function(K, W, X, Y) {
  var $ = W[0], B = W[1], Q = W[2], Z = W[3];
  return K[0] = $ + Y * (X[0] - $), K[1] = B + Y * (X[1] - B), K[2] = Q + Y * (X[2] - Q), K[3] = Z + Y * (X[3] - Z), K;
};
var f0 = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2] && K[3] === W[3];
};
var w0 = function(K, W) {
  var X = K[0], Y = K[1], $ = K[2], B = K[3], Q = W[0], Z = W[1], O = W[2], G = W[3];
  return Math.abs(X - Q) <= T * Math.max(1, Math.abs(X), Math.abs(Q)) && Math.abs(Y - Z) <= T * Math.max(1, Math.abs(Y), Math.abs(Z)) && Math.abs($ - O) <= T * Math.max(1, Math.abs($), Math.abs(O)) && Math.abs(B - G) <= T * Math.max(1, Math.abs(B), Math.abs(G));
};
var X0 = function() {
  var K = new _(4);
  if (_ != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0;
  return K[3] = 1, K;
};
var MW = function(K) {
  return K[0] = 0, K[1] = 0, K[2] = 0, K[3] = 1, K;
};
var d0 = function(K, W, X) {
  X = X * 0.5;
  var Y = Math.sin(X);
  return K[0] = Y * W[0], K[1] = Y * W[1], K[2] = Y * W[2], K[3] = Math.cos(X), K;
};
var FW = function(K, W) {
  var X = Math.acos(W[3]) * 2, Y = Math.sin(X / 2);
  if (Y > T)
    K[0] = W[0] / Y, K[1] = W[1] / Y, K[2] = W[2] / Y;
  else
    K[0] = 1, K[1] = 0, K[2] = 0;
  return X;
};
var pW = function(K, W) {
  var X = y0(K, W);
  return Math.acos(2 * X * X - 1);
};
var v0 = function(K, W, X) {
  var Y = W[0], $ = W[1], B = W[2], Q = W[3], Z = X[0], O = X[1], G = X[2], H = X[3];
  return K[0] = Y * H + Q * Z + $ * G - B * O, K[1] = $ * H + Q * O + B * Z - Y * G, K[2] = B * H + Q * G + Y * O - $ * Z, K[3] = Q * H - Y * Z - $ * O - B * G, K;
};
var gW = function(K, W, X) {
  X *= 0.5;
  var Y = W[0], $ = W[1], B = W[2], Q = W[3], Z = Math.sin(X), O = Math.cos(X);
  return K[0] = Y * O + Q * Z, K[1] = $ * O + B * Z, K[2] = B * O - $ * Z, K[3] = Q * O - Y * Z, K;
};
var qW = function(K, W, X) {
  X *= 0.5;
  var Y = W[0], $ = W[1], B = W[2], Q = W[3], Z = Math.sin(X), O = Math.cos(X);
  return K[0] = Y * O - B * Z, K[1] = $ * O + Q * Z, K[2] = B * O + Y * Z, K[3] = Q * O - $ * Z, K;
};
var fW = function(K, W, X) {
  X *= 0.5;
  var Y = W[0], $ = W[1], B = W[2], Q = W[3], Z = Math.sin(X), O = Math.cos(X);
  return K[0] = Y * O + $ * Z, K[1] = $ * O - Y * Z, K[2] = B * O + Q * Z, K[3] = Q * O - B * Z, K;
};
var wW = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2];
  return K[0] = X, K[1] = Y, K[2] = $, K[3] = Math.sqrt(Math.abs(1 - X * X - Y * Y - $ * $)), K;
};
var n0 = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], B = W[3], Q = Math.sqrt(X * X + Y * Y + $ * $), Z = Math.exp(B), O = Q > 0 ? Z * Math.sin(Q) / Q : 0;
  return K[0] = X * O, K[1] = Y * O, K[2] = $ * O, K[3] = Z * Math.cos(Q), K;
};
var l0 = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], B = W[3], Q = Math.sqrt(X * X + Y * Y + $ * $), Z = Q > 0 ? Math.atan2(Q, B) / Q : 0;
  return K[0] = X * Z, K[1] = Y * Z, K[2] = $ * Z, K[3] = 0.5 * Math.log(X * X + Y * Y + $ * $ + B * B), K;
};
var dW = function(K, W, X) {
  return l0(K, W), i0(K, K, X), n0(K, K), K;
};
var o = function(K, W, X, Y) {
  var $ = W[0], B = W[1], Q = W[2], Z = W[3], O = X[0], G = X[1], H = X[2], L = X[3], C, V, D, E, J;
  if (V = $ * O + B * G + Q * H + Z * L, V < 0)
    V = -V, O = -O, G = -G, H = -H, L = -L;
  if (1 - V > T)
    C = Math.acos(V), D = Math.sin(C), E = Math.sin((1 - Y) * C) / D, J = Math.sin(Y * C) / D;
  else
    E = 1 - Y, J = Y;
  return K[0] = E * $ + J * O, K[1] = E * B + J * G, K[2] = E * Q + J * H, K[3] = E * Z + J * L, K;
};
var vW = function(K) {
  var W = v(), X = v(), Y = v(), $ = Math.sqrt(1 - W), B = Math.sqrt(W);
  return K[0] = $ * Math.sin(2 * Math.PI * X), K[1] = $ * Math.cos(2 * Math.PI * X), K[2] = B * Math.sin(2 * Math.PI * Y), K[3] = B * Math.cos(2 * Math.PI * Y), K;
};
var nW = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], B = W[3], Q = X * X + Y * Y + $ * $ + B * B, Z = Q ? 1 / Q : 0;
  return K[0] = -X * Z, K[1] = -Y * Z, K[2] = -$ * Z, K[3] = B * Z, K;
};
var lW = function(K, W) {
  return K[0] = -W[0], K[1] = -W[1], K[2] = -W[2], K[3] = W[3], K;
};
var c0 = function(K, W) {
  var X = W[0] + W[4] + W[8], Y;
  if (X > 0)
    Y = Math.sqrt(X + 1), K[3] = 0.5 * Y, Y = 0.5 / Y, K[0] = (W[5] - W[7]) * Y, K[1] = (W[6] - W[2]) * Y, K[2] = (W[1] - W[3]) * Y;
  else {
    var $ = 0;
    if (W[4] > W[0])
      $ = 1;
    if (W[8] > W[$ * 3 + $])
      $ = 2;
    var B = ($ + 1) % 3, Q = ($ + 2) % 3;
    Y = Math.sqrt(W[$ * 3 + $] - W[B * 3 + B] - W[Q * 3 + Q] + 1), K[$] = 0.5 * Y, Y = 0.5 / Y, K[3] = (W[B * 3 + Q] - W[Q * 3 + B]) * Y, K[B] = (W[B * 3 + $] + W[$ * 3 + B]) * Y, K[Q] = (W[Q * 3 + $] + W[$ * 3 + Q]) * Y;
  }
  return K;
};
var cW = function(K, W, X, Y) {
  var $ = 0.5 * Math.PI / 180;
  W *= $, X *= $, Y *= $;
  var B = Math.sin(W), Q = Math.cos(W), Z = Math.sin(X), O = Math.cos(X), G = Math.sin(Y), H = Math.cos(Y);
  return K[0] = B * O * H - Q * Z * G, K[1] = Q * Z * H + B * O * G, K[2] = Q * O * G - B * Z * H, K[3] = Q * O * H + B * Z * G, K;
};
var iW = function(K) {
  return "quat(" + K[0] + ", " + K[1] + ", " + K[2] + ", " + K[3] + ")";
};
var a0 = Object.defineProperty;
var a = (K, W) => {
  for (var X in W)
    a0(K, X, { get: W[X], enumerable: true, configurable: true, set: (Y) => W[X] = () => Y });
};
var T = 0.000001;
var _ = typeof Float32Array !== "undefined" ? Float32Array : Array;
var v = Math.random;
var HX = Math.PI / 180;
if (!Math.hypot)
  Math.hypot = function() {
    var K = 0, W = arguments.length;
    while (W--)
      K += arguments[W] * arguments[W];
    return Math.sqrt(K);
  };
var S = {};
a(S, { transpose: () => {
  {
    return ZK;
  }
}, translate: () => {
  {
    return JK;
  }
}, targetTo: () => {
  {
    return dK;
  }
}, subtract: () => {
  {
    return E0;
  }
}, sub: () => {
  {
    return rK;
  }
}, str: () => {
  {
    return vK;
  }
}, set: () => {
  {
    return QK;
  }
}, scale: () => {
  {
    return LK;
  }
}, rotateZ: () => {
  {
    return DK;
  }
}, rotateY: () => {
  {
    return EK;
  }
}, rotateX: () => {
  {
    return CK;
  }
}, rotate: () => {
  {
    return VK;
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
    return gK;
  }
}, perspective: () => {
  {
    return FK;
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
    return iK;
  }
}, multiplyScalar: () => {
  {
    return cK;
  }
}, multiply: () => {
  {
    return H0;
  }
}, mul: () => {
  {
    return sK;
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
    return G0;
  }
}, getTranslation: () => {
  {
    return kK;
  }
}, getScaling: () => {
  {
    return L0;
  }
}, getRotation: () => {
  {
    return SK;
  }
}, frustum: () => {
  {
    return MK;
  }
}, fromZRotation: () => {
  {
    return TK;
  }
}, fromYRotation: () => {
  {
    return RK;
  }
}, fromXRotation: () => {
  {
    return AK;
  }
}, fromValues: () => {
  {
    return BK;
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
    return NK;
  }
}, fromQuat: () => {
  {
    return hK;
  }
}, frob: () => {
  {
    return nK;
  }
}, exactEquals: () => {
  {
    return yK;
  }
}, equals: () => {
  {
    return zK;
  }
}, determinant: () => {
  {
    return HK;
  }
}, create: () => {
  {
    return XK;
  }
}, copy: () => {
  {
    return $K;
  }
}, clone: () => {
  {
    return YK;
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
var FK = V0;
var qK = C0;
var sK = H0;
var rK = E0;
var m = {};
a(m, { str: () => {
  {
    return iW;
  }
}, squaredLength: () => {
  {
    return s0;
  }
}, sqrLen: () => {
  {
    return uW;
  }
}, sqlerp: () => {
  {
    return KX;
  }
}, slerp: () => {
  {
    return o;
  }
}, setAxisAngle: () => {
  {
    return d0;
  }
}, setAxes: () => {
  {
    return WX;
  }
}, set: () => {
  {
    return rW;
  }
}, scale: () => {
  {
    return i0;
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
    return gW;
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
    return Y0;
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
    return l0;
  }
}, lerp: () => {
  {
    return eW;
  }
}, length: () => {
  {
    return z0;
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
    return FW;
  }
}, getAngle: () => {
  {
    return pW;
  }
}, fromValues: () => {
  {
    return zW;
  }
}, fromMat3: () => {
  {
    return c0;
  }
}, fromEuler: () => {
  {
    return cW;
  }
}, exp: () => {
  {
    return n0;
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
    return y0;
  }
}, create: () => {
  {
    return X0;
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
var r = {};
a(r, { zero: () => {
  {
    return DW;
  }
}, transformQuat: () => {
  {
    return JW;
  }
}, transformMat4: () => {
  {
    return GW;
  }
}, transformMat3: () => {
  {
    return HW;
  }
}, subtract: () => {
  {
    return I0;
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
    return T0;
  }
}, squaredDistance: () => {
  {
    return R0;
  }
}, sqrLen: () => {
  {
    return SW;
  }
}, sqrDist: () => {
  {
    return kW;
  }
}, set: () => {
  {
    return eK;
  }
}, scaleAndAdd: () => {
  {
    return XW;
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
    return CW;
  }
}, rotateY: () => {
  {
    return VW;
  }
}, rotateX: () => {
  {
    return LW;
  }
}, random: () => {
  {
    return OW;
  }
}, normalize: () => {
  {
    return K0;
  }
}, negate: () => {
  {
    return YW;
  }
}, multiply: () => {
  {
    return U0;
  }
}, mul: () => {
  {
    return RW;
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
    return BW;
  }
}, length: () => {
  {
    return D0;
  }
}, len: () => {
  {
    return W0;
  }
}, inverse: () => {
  {
    return $W;
  }
}, hermite: () => {
  {
    return QW;
  }
}, fromValues: () => {
  {
    return b;
  }
}, forEach: () => {
  {
    return _W;
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
    return PW;
  }
}, dot: () => {
  {
    return u;
  }
}, divide: () => {
  {
    return P0;
  }
}, div: () => {
  {
    return TW;
  }
}, distance: () => {
  {
    return A0;
  }
}, dist: () => {
  {
    return NW;
  }
}, cross: () => {
  {
    return s;
  }
}, create: () => {
  {
    return e;
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
var AW = I0;
var RW = U0;
var TW = P0;
var NW = A0;
var kW = R0;
var W0 = D0;
var SW = T0;
var _W = function() {
  var K = e();
  return function(W, X, Y, $, B, Q) {
    var Z, O;
    if (!X)
      X = 3;
    if (!Y)
      Y = 0;
    if ($)
      O = Math.min($ * X + Y, W.length);
    else
      O = W.length;
    for (Z = Y;Z < O; Z += X)
      K[0] = W[Z], K[1] = W[Z + 1], K[2] = W[Z + 2], B(K, K, Q), W[Z] = K[0], W[Z + 1] = K[1], W[Z + 2] = K[2];
    return W;
  };
}();
var JX = function() {
  var K = jW();
  return function(W, X, Y, $, B, Q) {
    var Z, O;
    if (!X)
      X = 4;
    if (!Y)
      Y = 0;
    if ($)
      O = Math.min($ * X + Y, W.length);
    else
      O = W.length;
    for (Z = Y;Z < O; Z += X)
      K[0] = W[Z], K[1] = W[Z + 1], K[2] = W[Z + 2], K[3] = W[Z + 3], B(K, K, Q), W[Z] = K[0], W[Z + 1] = K[1], W[Z + 2] = K[2], W[Z + 3] = K[3];
    return W;
  };
}();
var yW = N0;
var zW = k0;
var sW = S0;
var rW = _0;
var mW = j0;
var xW = v0;
var i0 = h0;
var y0 = g0;
var eW = q0;
var z0 = M0;
var bW = z0;
var s0 = F0;
var uW = s0;
var Y0 = p0;
var oW = f0;
var tW = w0;
var aW = function() {
  var K = e(), W = b(1, 0, 0), X = b(0, 1, 0);
  return function(Y, $, B) {
    var Q = u($, B);
    if (Q < -0.999999) {
      if (s(K, W, $), W0(K) < 0.000001)
        s(K, X, $);
      return K0(K, K), d0(Y, K, Math.PI), Y;
    } else if (Q > 0.999999)
      return Y[0] = 0, Y[1] = 0, Y[2] = 0, Y[3] = 1, Y;
    else
      return s(K, $, B), Y[0] = K[0], Y[1] = K[1], Y[2] = K[2], Y[3] = 1 + Q, Y0(Y, Y);
  };
}();
var KX = function() {
  var K = X0(), W = X0();
  return function(X, Y, $, B, Q, Z) {
    return o(K, Y, Q, Z), o(W, $, B, Z), o(X, K, W, 2 * Z * (1 - Z)), X;
  };
}();
var WX = function() {
  var K = O0();
  return function(W, X, Y, $) {
    return K[0] = Y[0], K[3] = Y[1], K[6] = Y[2], K[1] = $[0], K[4] = $[1], K[7] = $[2], K[2] = -X[0], K[5] = -X[1], K[8] = -X[2], Y0(W, c0(W, K));
  };
}();

class n {
  d;
  listeners = new Set;
  constructor(K) {
    this.elem = K;
  }
  addChangeListener(K) {
    return this.listeners.add(K), this;
  }
  removeChangeListener(K) {
    this.listeners.delete(K);
  }
  onChange() {
    for (let K of this.listeners)
      K.onChange(this.elem);
  }
}
var XX = Math.PI / 90;
var $0 = [0, 0, 0];
var r0 = S.create();
var m0 = S.create();
var t = m.create();

class x {
  static HIDDEN = x.create().scale(0, 0, 0);
  static IDENTITY = x.create();
  #K = Float32Array.from(S.create());
  #W = new n(this);
  constructor() {
    this.identity();
  }
  addChangeListener(K) {
    return this.#W.addChangeListener(K), this;
  }
  removeChangeListener(K) {
    this.#W.removeChangeListener(K);
  }
  static create() {
    return new x;
  }
  copy(K) {
    return S.copy(this.#K, K.getMatrix()), this.#W.onChange(), this;
  }
  identity() {
    return S.identity(this.#K), this.#W.onChange(), this;
  }
  invert(K) {
    return S.invert(this.#K, K?.getMatrix() ?? this.getMatrix()), this.#W.onChange(), this;
  }
  multiply(K) {
    return S.multiply(this.#K, this.#K, K.getMatrix()), this.#W.onChange(), this;
  }
  multiply2(K, W) {
    return S.multiply(this.#K, K.getMatrix(), W.getMatrix()), this.#W.onChange(), this;
  }
  multiply3(K, W, X) {
    return this.multiply2(K, W), this.multiply(X), this;
  }
  translate(K, W, X) {
    const Y = $0;
    return Y[0] = K, Y[1] = W, Y[2] = X, this.move(Y);
  }
  move(K) {
    return S.translate(this.#K, this.#K, K), this.#W.onChange(), this;
  }
  rotateX(K) {
    return S.rotateX(this.#K, this.#K, K), this.#W.onChange(), this;
  }
  rotateY(K) {
    return S.rotateY(this.#K, this.#K, K), this.#W.onChange(), this;
  }
  rotateZ(K) {
    return S.rotateZ(this.#K, this.#K, K), this.#W.onChange(), this;
  }
  setXRotation(K) {
    return S.fromXRotation(this.getMatrix(), K), this.#W.onChange(), this;
  }
  setYRotation(K) {
    return S.fromYRotation(this.getMatrix(), K), this.#W.onChange(), this;
  }
  scale(K, W, X) {
    return S.scale(this.#K, this.#K, [K, W ?? K, X ?? K]), this.#W.onChange(), this;
  }
  perspective(K, W, X, Y) {
    return S.perspective(this.#K, K * XX, W, X, Y), this.#W.onChange(), this;
  }
  ortho(K, W, X, Y, $, B) {
    return S.ortho(this.#K, K, W, X, Y, $, B), this.#W.onChange(), this;
  }
  combine(K, W, X = 0.5) {
    return S.multiplyScalar(r0, K.getMatrix(), 1 - X), S.multiplyScalar(m0, W.getMatrix(), X), S.add(this.#K, r0, m0), this.#W.onChange(), this;
  }
  static getMoveVector(K, W, X, Y) {
    const $ = $0;
    if ($[0] = K, $[1] = W, $[2] = X, Y)
      S.getRotation(t, Y.getMatrix()), m.invert(t, t), r.transformQuat($, $, t);
    return $;
  }
  getPosition() {
    const K = $0;
    return K[0] = this.#K[12], K[1] = this.#K[13], K[2] = this.#K[14], K;
  }
  setVector(K) {
    return this.setPosition(K[0], K[1], K[2]);
  }
  setPosition(K, W, X) {
    return this.#K[12] = K, this.#K[13] = W, this.#K[14] = X, this.#W.onChange(), this;
  }
  getMatrix() {
    return this.#K;
  }
}
var w = x;

class x0 {
  w;
  q;
  #K;
  #W = false;
  #X = 0;
  #Y;
  #$;
  constructor(K, W, X) {
    this.getValue = W, this.apply = X, this.#$ = K, this.#K = this.getValue(K);
  }
  set element(K) {
    this.#$ = K, this.#K = this.getValue(K), this.#Y = undefined;
  }
  setGoal(K, W, X) {
    if (this.#Y && this.#Y !== X)
      return;
    if (this.#K !== K || this.#X !== W)
      this.#X = W, this.#K = K, this.#Y = X, this.#W = true;
  }
  get goal() {
    return this.#K;
  }
  update(K) {
    if (this.#W) {
      const W = this.getValue(this.#$), X = this.goal - W, Y = Math.min(Math.abs(X), this.#X * K);
      if (Y <= 0.01)
        this.apply(this.#$, this.goal), this.#W = false, this.#Y = undefined;
      else
        this.apply(this.#$, W + Y * Math.sign(X));
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
    const X = this.initCall(undefined, ...K);
    return this.#K.add(X), this.#Y(), X;
  }
  recycle(K) {
    this.#K.delete(K), this.#X(K);
  }
  recycleAll() {
    for (let K of this.#K)
      this.#X(K);
    this.#K.clear();
  }
  clear() {
    this.#W.length = 0, this.#K.clear();
  }
  countObjectsInExistence() {
    return this.#K.size + this.#W.length;
  }
  #X(K) {
    this.#W.push(K), this.onRecycle?.(K);
  }
  #Y() {
    if (this.countObjectsInExistence() === this.warningLimit)
      console.warn("ObjectPool already created", this.#K.size + this.#W.length, "in", this.constructor.name);
  }
}

class b0 extends e0 {
  constructor() {
    super((K, W) => {
      if (!K)
        return new x0(W, (X) => X.valueOf(), (X, Y) => X.setValue(Y));
      return K.element = W, K;
    });
  }
}
var YX = new b0;

class l {
  w;
  q;
  #K = 0;
  #W;
  constructor(K = 0, W, X = YX) {
    this.onChange = W, this.pool = X, this.#K = K;
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
  progressTowards(K, W, X, Y) {
    if (!this.#W)
      this.#W = this.pool.create(this);
    if (this.#W.setGoal(K, W, X), Y)
      Y.loop(this, undefined);
  }
  get goal() {
    return this.#W?.goal ?? this.valueOf();
  }
}
class c {
  static toVector(K, W, X, Y) {
    return Y[0] = K, Y[1] = W, Y[2] = X, Y;
  }
  static transformToPosition(K, W) {
    const X = K.getMatrix();
    W[0] = X[12], W[1] = X[13], W[2] = X[14];
  }
}
var d;
(function(Y) {
  Y[Y["AT_POSITION"] = 0] = "AT_POSITION";
  Y[Y["MOVED"] = 1] = "MOVED";
  Y[Y["BLOCKED"] = 2] = "BLOCKED";
})(d || (d = {}));
var B0 = function(K, W) {
  if (K) {
    const X = K.length.valueOf();
    for (let Y = 0;Y < X; Y++) {
      const $ = K.at(Y);
      if ($ !== undefined && W($, Y))
        return true;
    }
  }
  return false;
};

class o0 {
  #K = w.create().setPosition(0, 0, 0);
  #W = [0, 0, 0];
  #X = new n(this);
  position = [0, 0, 0];
  blockers;
  constructor({ blockers: K } = {}) {
    this.blockers = K, this.#K.addChangeListener({ onChange: () => {
      c.transformToPosition(this.#K, this.position), this.#X.onChange();
    } });
  }
  addChangeListener(K) {
    return this.#X.addChangeListener(K), this;
  }
  removeChangeListener(K) {
    this.#X.removeChangeListener(K);
  }
  moveBy(K, W, X, Y) {
    const $ = w.getMoveVector(K, W, X, Y), B = B0(this.blockers, (Q) => Q.isBlocked(c.toVector(this.position[0] + $[0], this.position[1] + $[1], this.position[2] + $[2], this.#W), this.position));
    if (!B)
      if ($[0] || $[1] || $[2])
        this.#K.move($);
      else
        return d.AT_POSITION;
    return B ? d.BLOCKED : d.MOVED;
  }
  moveTo(K, W, X) {
    if (this.position[0] === K && this.position[1] === W && this.position[2] === X)
      return d.AT_POSITION;
    const Y = B0(this.blockers, ($) => $.isBlocked(c.toVector(K, W, X, this.#W), this.position));
    if (!Y) {
      const [$, B, Q] = this.#K.getPosition();
      if ($ !== K || B !== W || Q !== X)
        this.#K.setPosition(K, W, X);
    }
    return Y ? d.BLOCKED : d.MOVED;
  }
  movedTo(K, W, X) {
    return this.moveTo(K, W, X), this;
  }
  gotoPos(K, W, X, Y = 0.1) {
    const $ = this.position, B = K - $[0], Q = W - $[1], Z = X - $[2], O = Math.sqrt(B * B + Q * Q + Z * Z);
    if (O > 0.01) {
      const G = Math.min(O, Y);
      return this.moveBy(B / O * G, Q / O * G, Z / O * G);
    } else
      return this.moveTo(K, W, X);
  }
  getMatrix() {
    return this.#K.getMatrix();
  }
}
var QX = 1;
var ZX = 1;

class t0 {
  #K = w.create();
  #W = w.create();
  #X = w.create();
  #Y = [0, 0];
  #$ = new n(this);
  perspective;
  zoom;
  constructor() {
    const K = { onChange: () => {
      this.#K.combine(this.#X, this.#W, this.perspective.valueOf());
    } };
    this.perspective = new l(QX, K.onChange), this.zoom = new l(ZX, (W) => {
      this.configure(this.#Y, W);
    }), this.#W.addChangeListener(K), this.#X.addChangeListener(K), this.#K.addChangeListener(this.#$);
  }
  addChangeListener(K) {
    return this.#$.addChangeListener(K), this;
  }
  removeChangeListener(K) {
    this.#$.removeChangeListener(K);
  }
  configPerspectiveMatrix(K, W, X, Y) {
    this.#W.perspective(K, W, X, Y);
  }
  configOrthoMatrix(K, W, X, Y) {
    this.#X.ortho(-K / 2, K / 2, -W / 2, W / 2, X, Y);
  }
  configure(K, W, X = 0.5, Y = 1e4) {
    if (!W)
      W = this.zoom.valueOf();
    this.#Y[0] = K[0], this.#Y[1] = K[1];
    const $ = this.#Y[0] / this.#Y[1], B = 45 / Math.sqrt(W);
    this.configPerspectiveMatrix(B, $, Math.max(X, 0.00001), Y), this.configOrthoMatrix($ / W / W, 1 / W / W, -Y, Y);
  }
  getMatrix() {
    return this.#K.getMatrix();
  }
}
export {
  w as Matrix
};
