// /Users/vincent/dok-matrix/example/node_modules/dok-matrix/dist/index.js
var J0 = function() {
  var K = new k(9);
  if (k != Float32Array)
    K[1] = 0, K[2] = 0, K[3] = 0, K[5] = 0, K[6] = 0, K[7] = 0;
  return K[0] = 1, K[4] = 1, K[8] = 1, K;
};
var $K = function() {
  var K = new k(16);
  if (k != Float32Array)
    K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0;
  return K[0] = 1, K[5] = 1, K[10] = 1, K[15] = 1, K;
};
var QK = function(K) {
  var W = new k(16);
  return W[0] = K[0], W[1] = K[1], W[2] = K[2], W[3] = K[3], W[4] = K[4], W[5] = K[5], W[6] = K[6], W[7] = K[7], W[8] = K[8], W[9] = K[9], W[10] = K[10], W[11] = K[11], W[12] = K[12], W[13] = K[13], W[14] = K[14], W[15] = K[15], W;
};
var ZK = function(K, W) {
  return K[0] = W[0], K[1] = W[1], K[2] = W[2], K[3] = W[3], K[4] = W[4], K[5] = W[5], K[6] = W[6], K[7] = W[7], K[8] = W[8], K[9] = W[9], K[10] = W[10], K[11] = W[11], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15], K;
};
var BK = function(K, W, X, Y, $, Q, Z, B, O, J, G, L, C, V, D, E) {
  var H = new k(16);
  return H[0] = K, H[1] = W, H[2] = X, H[3] = Y, H[4] = $, H[5] = Q, H[6] = Z, H[7] = B, H[8] = O, H[9] = J, H[10] = G, H[11] = L, H[12] = C, H[13] = V, H[14] = D, H[15] = E, H;
};
var OK = function(K, W, X, Y, $, Q, Z, B, O, J, G, L, C, V, D, E, H) {
  return K[0] = W, K[1] = X, K[2] = Y, K[3] = $, K[4] = Q, K[5] = Z, K[6] = B, K[7] = O, K[8] = J, K[9] = G, K[10] = L, K[11] = C, K[12] = V, K[13] = D, K[14] = E, K[15] = H, K;
};
var G0 = function(K) {
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var JK = function(K, W) {
  if (K === W) {
    var X = W[1], Y = W[2], $ = W[3], Q = W[6], Z = W[7], B = W[11];
    K[1] = W[4], K[2] = W[8], K[3] = W[12], K[4] = X, K[6] = W[9], K[7] = W[13], K[8] = Y, K[9] = Q, K[11] = W[14], K[12] = $, K[13] = Z, K[14] = B;
  } else
    K[0] = W[0], K[1] = W[4], K[2] = W[8], K[3] = W[12], K[4] = W[1], K[5] = W[5], K[6] = W[9], K[7] = W[13], K[8] = W[2], K[9] = W[6], K[10] = W[10], K[11] = W[14], K[12] = W[3], K[13] = W[7], K[14] = W[11], K[15] = W[15];
  return K;
};
var GK = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q = W[3], Z = W[4], B = W[5], O = W[6], J = W[7], G = W[8], L = W[9], C = W[10], V = W[11], D = W[12], E = W[13], H = W[14], I = W[15], T = X * B - Y * Z, R = X * O - $ * Z, P = X * J - Q * Z, U = Y * O - $ * B, A = Y * J - Q * B, p = $ * J - Q * O, h = G * E - L * D, j = G * H - C * D, M = G * I - V * D, F = L * H - C * E, q = L * I - V * E, g = C * I - V * H, S = T * g - R * q + P * F + U * M - A * j + p * h;
  if (!S)
    return null;
  return S = 1 / S, K[0] = (B * g - O * q + J * F) * S, K[1] = ($ * q - Y * g - Q * F) * S, K[2] = (E * p - H * A + I * U) * S, K[3] = (C * A - L * p - V * U) * S, K[4] = (O * M - Z * g - J * j) * S, K[5] = (X * g - $ * M + Q * j) * S, K[6] = (H * P - D * p - I * R) * S, K[7] = (G * p - C * P + V * R) * S, K[8] = (Z * q - B * M + J * h) * S, K[9] = (Y * M - X * q - Q * h) * S, K[10] = (D * A - E * P + I * T) * S, K[11] = (L * P - G * A - V * T) * S, K[12] = (B * j - Z * F - O * h) * S, K[13] = (X * F - Y * j + $ * h) * S, K[14] = (E * R - D * U - H * T) * S, K[15] = (G * U - L * R + C * T) * S, K;
};
var HK = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q = W[3], Z = W[4], B = W[5], O = W[6], J = W[7], G = W[8], L = W[9], C = W[10], V = W[11], D = W[12], E = W[13], H = W[14], I = W[15];
  return K[0] = B * (C * I - V * H) - L * (O * I - J * H) + E * (O * V - J * C), K[1] = -(Y * (C * I - V * H) - L * ($ * I - Q * H) + E * ($ * V - Q * C)), K[2] = Y * (O * I - J * H) - B * ($ * I - Q * H) + E * ($ * J - Q * O), K[3] = -(Y * (O * V - J * C) - B * ($ * V - Q * C) + L * ($ * J - Q * O)), K[4] = -(Z * (C * I - V * H) - G * (O * I - J * H) + D * (O * V - J * C)), K[5] = X * (C * I - V * H) - G * ($ * I - Q * H) + D * ($ * V - Q * C), K[6] = -(X * (O * I - J * H) - Z * ($ * I - Q * H) + D * ($ * J - Q * O)), K[7] = X * (O * V - J * C) - Z * ($ * V - Q * C) + G * ($ * J - Q * O), K[8] = Z * (L * I - V * E) - G * (B * I - J * E) + D * (B * V - J * L), K[9] = -(X * (L * I - V * E) - G * (Y * I - Q * E) + D * (Y * V - Q * L)), K[10] = X * (B * I - J * E) - Z * (Y * I - Q * E) + D * (Y * J - Q * B), K[11] = -(X * (B * V - J * L) - Z * (Y * V - Q * L) + G * (Y * J - Q * B)), K[12] = -(Z * (L * H - C * E) - G * (B * H - O * E) + D * (B * C - O * L)), K[13] = X * (L * H - C * E) - G * (Y * H - $ * E) + D * (Y * C - $ * L), K[14] = -(X * (B * H - O * E) - Z * (Y * H - $ * E) + D * (Y * O - $ * B)), K[15] = X * (B * C - O * L) - Z * (Y * C - $ * L) + G * (Y * O - $ * B), K;
};
var LK = function(K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3], Q = K[4], Z = K[5], B = K[6], O = K[7], J = K[8], G = K[9], L = K[10], C = K[11], V = K[12], D = K[13], E = K[14], H = K[15], I = W * Z - X * Q, T = W * B - Y * Q, R = W * O - $ * Q, P = X * B - Y * Z, U = X * O - $ * Z, A = Y * O - $ * B, p = J * D - G * V, h = J * E - L * V, j = J * H - C * V, M = G * E - L * D, F = G * H - C * D, q = L * H - C * E;
  return I * q - T * F + R * M + P * j - U * h + A * p;
};
var H0 = function(K, W, X) {
  var Y = W[0], $ = W[1], Q = W[2], Z = W[3], B = W[4], O = W[5], J = W[6], G = W[7], L = W[8], C = W[9], V = W[10], D = W[11], E = W[12], H = W[13], I = W[14], T = W[15], R = X[0], P = X[1], U = X[2], A = X[3];
  return K[0] = R * Y + P * B + U * L + A * E, K[1] = R * $ + P * O + U * C + A * H, K[2] = R * Q + P * J + U * V + A * I, K[3] = R * Z + P * G + U * D + A * T, R = X[4], P = X[5], U = X[6], A = X[7], K[4] = R * Y + P * B + U * L + A * E, K[5] = R * $ + P * O + U * C + A * H, K[6] = R * Q + P * J + U * V + A * I, K[7] = R * Z + P * G + U * D + A * T, R = X[8], P = X[9], U = X[10], A = X[11], K[8] = R * Y + P * B + U * L + A * E, K[9] = R * $ + P * O + U * C + A * H, K[10] = R * Q + P * J + U * V + A * I, K[11] = R * Z + P * G + U * D + A * T, R = X[12], P = X[13], U = X[14], A = X[15], K[12] = R * Y + P * B + U * L + A * E, K[13] = R * $ + P * O + U * C + A * H, K[14] = R * Q + P * J + U * V + A * I, K[15] = R * Z + P * G + U * D + A * T, K;
};
var VK = function(K, W, X) {
  var Y = X[0], $ = X[1], Q = X[2], Z, B, O, J, G, L, C, V, D, E, H, I;
  if (W === K)
    K[12] = W[0] * Y + W[4] * $ + W[8] * Q + W[12], K[13] = W[1] * Y + W[5] * $ + W[9] * Q + W[13], K[14] = W[2] * Y + W[6] * $ + W[10] * Q + W[14], K[15] = W[3] * Y + W[7] * $ + W[11] * Q + W[15];
  else
    Z = W[0], B = W[1], O = W[2], J = W[3], G = W[4], L = W[5], C = W[6], V = W[7], D = W[8], E = W[9], H = W[10], I = W[11], K[0] = Z, K[1] = B, K[2] = O, K[3] = J, K[4] = G, K[5] = L, K[6] = C, K[7] = V, K[8] = D, K[9] = E, K[10] = H, K[11] = I, K[12] = Z * Y + G * $ + D * Q + W[12], K[13] = B * Y + L * $ + E * Q + W[13], K[14] = O * Y + C * $ + H * Q + W[14], K[15] = J * Y + V * $ + I * Q + W[15];
  return K;
};
var CK = function(K, W, X) {
  var Y = X[0], $ = X[1], Q = X[2];
  return K[0] = W[0] * Y, K[1] = W[1] * Y, K[2] = W[2] * Y, K[3] = W[3] * Y, K[4] = W[4] * $, K[5] = W[5] * $, K[6] = W[6] * $, K[7] = W[7] * $, K[8] = W[8] * Q, K[9] = W[9] * Q, K[10] = W[10] * Q, K[11] = W[11] * Q, K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15], K;
};
var EK = function(K, W, X, Y) {
  var $ = Y[0], Q = Y[1], Z = Y[2], B = Math.hypot($, Q, Z), O, J, G, L, C, V, D, E, H, I, T, R, P, U, A, p, h, j, M, F, q, g, S, f;
  if (B < N)
    return null;
  if (B = 1 / B, $ *= B, Q *= B, Z *= B, O = Math.sin(X), J = Math.cos(X), G = 1 - J, L = W[0], C = W[1], V = W[2], D = W[3], E = W[4], H = W[5], I = W[6], T = W[7], R = W[8], P = W[9], U = W[10], A = W[11], p = $ * $ * G + J, h = Q * $ * G + Z * O, j = Z * $ * G - Q * O, M = $ * Q * G - Z * O, F = Q * Q * G + J, q = Z * Q * G + $ * O, g = $ * Z * G + Q * O, S = Q * Z * G - $ * O, f = Z * Z * G + J, K[0] = L * p + E * h + R * j, K[1] = C * p + H * h + P * j, K[2] = V * p + I * h + U * j, K[3] = D * p + T * h + A * j, K[4] = L * M + E * F + R * q, K[5] = C * M + H * F + P * q, K[6] = V * M + I * F + U * q, K[7] = D * M + T * F + A * q, K[8] = L * g + E * S + R * f, K[9] = C * g + H * S + P * f, K[10] = V * g + I * S + U * f, K[11] = D * g + T * S + A * f, W !== K)
    K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K;
};
var DK = function(K, W, X) {
  var Y = Math.sin(X), $ = Math.cos(X), Q = W[4], Z = W[5], B = W[6], O = W[7], J = W[8], G = W[9], L = W[10], C = W[11];
  if (W !== K)
    K[0] = W[0], K[1] = W[1], K[2] = W[2], K[3] = W[3], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[4] = Q * $ + J * Y, K[5] = Z * $ + G * Y, K[6] = B * $ + L * Y, K[7] = O * $ + C * Y, K[8] = J * $ - Q * Y, K[9] = G * $ - Z * Y, K[10] = L * $ - B * Y, K[11] = C * $ - O * Y, K;
};
var IK = function(K, W, X) {
  var Y = Math.sin(X), $ = Math.cos(X), Q = W[0], Z = W[1], B = W[2], O = W[3], J = W[8], G = W[9], L = W[10], C = W[11];
  if (W !== K)
    K[4] = W[4], K[5] = W[5], K[6] = W[6], K[7] = W[7], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[0] = Q * $ - J * Y, K[1] = Z * $ - G * Y, K[2] = B * $ - L * Y, K[3] = O * $ - C * Y, K[8] = Q * Y + J * $, K[9] = Z * Y + G * $, K[10] = B * Y + L * $, K[11] = O * Y + C * $, K;
};
var UK = function(K, W, X) {
  var Y = Math.sin(X), $ = Math.cos(X), Q = W[0], Z = W[1], B = W[2], O = W[3], J = W[4], G = W[5], L = W[6], C = W[7];
  if (W !== K)
    K[8] = W[8], K[9] = W[9], K[10] = W[10], K[11] = W[11], K[12] = W[12], K[13] = W[13], K[14] = W[14], K[15] = W[15];
  return K[0] = Q * $ + J * Y, K[1] = Z * $ + G * Y, K[2] = B * $ + L * Y, K[3] = O * $ + C * Y, K[4] = J * $ - Q * Y, K[5] = G * $ - Z * Y, K[6] = L * $ - B * Y, K[7] = C * $ - O * Y, K;
};
var AK = function(K, W) {
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = W[0], K[13] = W[1], K[14] = W[2], K[15] = 1, K;
};
var PK = function(K, W) {
  return K[0] = W[0], K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = W[1], K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = W[2], K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var RK = function(K, W, X) {
  var Y = X[0], $ = X[1], Q = X[2], Z = Math.hypot(Y, $, Q), B, O, J;
  if (Z < N)
    return null;
  return Z = 1 / Z, Y *= Z, $ *= Z, Q *= Z, B = Math.sin(W), O = Math.cos(W), J = 1 - O, K[0] = Y * Y * J + O, K[1] = $ * Y * J + Q * B, K[2] = Q * Y * J - $ * B, K[3] = 0, K[4] = Y * $ * J - Q * B, K[5] = $ * $ * J + O, K[6] = Q * $ * J + Y * B, K[7] = 0, K[8] = Y * Q * J + $ * B, K[9] = $ * Q * J - Y * B, K[10] = Q * Q * J + O, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var NK = function(K, W) {
  var X = Math.sin(W), Y = Math.cos(W);
  return K[0] = 1, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = Y, K[6] = X, K[7] = 0, K[8] = 0, K[9] = -X, K[10] = Y, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var TK = function(K, W) {
  var X = Math.sin(W), Y = Math.cos(W);
  return K[0] = Y, K[1] = 0, K[2] = -X, K[3] = 0, K[4] = 0, K[5] = 1, K[6] = 0, K[7] = 0, K[8] = X, K[9] = 0, K[10] = Y, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var SK = function(K, W) {
  var X = Math.sin(W), Y = Math.cos(W);
  return K[0] = Y, K[1] = X, K[2] = 0, K[3] = 0, K[4] = -X, K[5] = Y, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 1, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var L0 = function(K, W, X) {
  var Y = W[0], $ = W[1], Q = W[2], Z = W[3], B = Y + Y, O = $ + $, J = Q + Q, G = Y * B, L = Y * O, C = Y * J, V = $ * O, D = $ * J, E = Q * J, H = Z * B, I = Z * O, T = Z * J;
  return K[0] = 1 - (V + E), K[1] = L + T, K[2] = C - I, K[3] = 0, K[4] = L - T, K[5] = 1 - (G + E), K[6] = D + H, K[7] = 0, K[8] = C + I, K[9] = D - H, K[10] = 1 - (G + V), K[11] = 0, K[12] = X[0], K[13] = X[1], K[14] = X[2], K[15] = 1, K;
};
var _K = function(K, W) {
  var X = new k(3), Y = -W[0], $ = -W[1], Q = -W[2], Z = W[3], B = W[4], O = W[5], J = W[6], G = W[7], L = Y * Y + $ * $ + Q * Q + Z * Z;
  if (L > 0)
    X[0] = (B * Z + G * Y + O * Q - J * $) * 2 / L, X[1] = (O * Z + G * $ + J * Y - B * Q) * 2 / L, X[2] = (J * Z + G * Q + B * $ - O * Y) * 2 / L;
  else
    X[0] = (B * Z + G * Y + O * Q - J * $) * 2, X[1] = (O * Z + G * $ + J * Y - B * Q) * 2, X[2] = (J * Z + G * Q + B * $ - O * Y) * 2;
  return L0(K, W, X), K;
};
var kK = function(K, W) {
  return K[0] = W[12], K[1] = W[13], K[2] = W[14], K;
};
var V0 = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q = W[4], Z = W[5], B = W[6], O = W[8], J = W[9], G = W[10];
  return K[0] = Math.hypot(X, Y, $), K[1] = Math.hypot(Q, Z, B), K[2] = Math.hypot(O, J, G), K;
};
var hK = function(K, W) {
  var X = new k(3);
  V0(X, W);
  var Y = 1 / X[0], $ = 1 / X[1], Q = 1 / X[2], Z = W[0] * Y, B = W[1] * $, O = W[2] * Q, J = W[4] * Y, G = W[5] * $, L = W[6] * Q, C = W[8] * Y, V = W[9] * $, D = W[10] * Q, E = Z + G + D, H = 0;
  if (E > 0)
    H = Math.sqrt(E + 1) * 2, K[3] = 0.25 * H, K[0] = (L - V) / H, K[1] = (C - O) / H, K[2] = (B - J) / H;
  else if (Z > G && Z > D)
    H = Math.sqrt(1 + Z - G - D) * 2, K[3] = (L - V) / H, K[0] = 0.25 * H, K[1] = (B + J) / H, K[2] = (C + O) / H;
  else if (G > D)
    H = Math.sqrt(1 + G - Z - D) * 2, K[3] = (C - O) / H, K[0] = (B + J) / H, K[1] = 0.25 * H, K[2] = (L + V) / H;
  else
    H = Math.sqrt(1 + D - Z - G) * 2, K[3] = (B - J) / H, K[0] = (C + O) / H, K[1] = (L + V) / H, K[2] = 0.25 * H;
  return K;
};
var jK = function(K, W, X, Y) {
  var $ = W[0], Q = W[1], Z = W[2], B = W[3], O = $ + $, J = Q + Q, G = Z + Z, L = $ * O, C = $ * J, V = $ * G, D = Q * J, E = Q * G, H = Z * G, I = B * O, T = B * J, R = B * G, P = Y[0], U = Y[1], A = Y[2];
  return K[0] = (1 - (D + H)) * P, K[1] = (C + R) * P, K[2] = (V - T) * P, K[3] = 0, K[4] = (C - R) * U, K[5] = (1 - (L + H)) * U, K[6] = (E + I) * U, K[7] = 0, K[8] = (V + T) * A, K[9] = (E - I) * A, K[10] = (1 - (L + D)) * A, K[11] = 0, K[12] = X[0], K[13] = X[1], K[14] = X[2], K[15] = 1, K;
};
var MK = function(K, W, X, Y, $) {
  var Q = W[0], Z = W[1], B = W[2], O = W[3], J = Q + Q, G = Z + Z, L = B + B, C = Q * J, V = Q * G, D = Q * L, E = Z * G, H = Z * L, I = B * L, T = O * J, R = O * G, P = O * L, U = Y[0], A = Y[1], p = Y[2], h = $[0], j = $[1], M = $[2], F = (1 - (E + I)) * U, q = (V + P) * U, g = (D - R) * U, S = (V - P) * A, f = (1 - (C + I)) * A, i = (H + T) * A, y = (D + R) * p, B0 = (H - T) * p, O0 = (1 - (C + E)) * p;
  return K[0] = F, K[1] = q, K[2] = g, K[3] = 0, K[4] = S, K[5] = f, K[6] = i, K[7] = 0, K[8] = y, K[9] = B0, K[10] = O0, K[11] = 0, K[12] = X[0] + h - (F * h + S * j + y * M), K[13] = X[1] + j - (q * h + f * j + B0 * M), K[14] = X[2] + M - (g * h + i * j + O0 * M), K[15] = 1, K;
};
var pK = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q = W[3], Z = X + X, B = Y + Y, O = $ + $, J = X * Z, G = Y * Z, L = Y * B, C = $ * Z, V = $ * B, D = $ * O, E = Q * Z, H = Q * B, I = Q * O;
  return K[0] = 1 - L - D, K[1] = G + I, K[2] = C - H, K[3] = 0, K[4] = G - I, K[5] = 1 - J - D, K[6] = V + E, K[7] = 0, K[8] = C + H, K[9] = V - E, K[10] = 1 - J - L, K[11] = 0, K[12] = 0, K[13] = 0, K[14] = 0, K[15] = 1, K;
};
var FK = function(K, W, X, Y, $, Q, Z) {
  var B = 1 / (X - W), O = 1 / ($ - Y), J = 1 / (Q - Z);
  return K[0] = Q * 2 * B, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = Q * 2 * O, K[6] = 0, K[7] = 0, K[8] = (X + W) * B, K[9] = ($ + Y) * O, K[10] = (Z + Q) * J, K[11] = -1, K[12] = 0, K[13] = 0, K[14] = Z * Q * 2 * J, K[15] = 0, K;
};
var C0 = function(K, W, X, Y, $) {
  var Q = 1 / Math.tan(W / 2), Z;
  if (K[0] = Q / X, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = Q, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = -1, K[12] = 0, K[13] = 0, K[15] = 0, $ != null && $ !== Infinity)
    Z = 1 / (Y - $), K[10] = ($ + Y) * Z, K[14] = 2 * $ * Y * Z;
  else
    K[10] = -1, K[14] = -2 * Y;
  return K;
};
var gK = function(K, W, X, Y, $) {
  var Q = 1 / Math.tan(W / 2), Z;
  if (K[0] = Q / X, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = Q, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[11] = -1, K[12] = 0, K[13] = 0, K[15] = 0, $ != null && $ !== Infinity)
    Z = 1 / (Y - $), K[10] = $ * Z, K[14] = $ * Y * Z;
  else
    K[10] = -1, K[14] = -Y;
  return K;
};
var fK = function(K, W, X, Y) {
  var $ = Math.tan(W.upDegrees * Math.PI / 180), Q = Math.tan(W.downDegrees * Math.PI / 180), Z = Math.tan(W.leftDegrees * Math.PI / 180), B = Math.tan(W.rightDegrees * Math.PI / 180), O = 2 / (Z + B), J = 2 / ($ + Q);
  return K[0] = O, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = J, K[6] = 0, K[7] = 0, K[8] = -((Z - B) * O * 0.5), K[9] = ($ - Q) * J * 0.5, K[10] = Y / (X - Y), K[11] = -1, K[12] = 0, K[13] = 0, K[14] = Y * X / (X - Y), K[15] = 0, K;
};
var E0 = function(K, W, X, Y, $, Q, Z) {
  var B = 1 / (W - X), O = 1 / (Y - $), J = 1 / (Q - Z);
  return K[0] = -2 * B, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = -2 * O, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = 2 * J, K[11] = 0, K[12] = (W + X) * B, K[13] = ($ + Y) * O, K[14] = (Z + Q) * J, K[15] = 1, K;
};
var dK = function(K, W, X, Y, $, Q, Z) {
  var B = 1 / (W - X), O = 1 / (Y - $), J = 1 / (Q - Z);
  return K[0] = -2 * B, K[1] = 0, K[2] = 0, K[3] = 0, K[4] = 0, K[5] = -2 * O, K[6] = 0, K[7] = 0, K[8] = 0, K[9] = 0, K[10] = J, K[11] = 0, K[12] = (W + X) * B, K[13] = ($ + Y) * O, K[14] = Q * J, K[15] = 1, K;
};
var vK = function(K, W, X, Y) {
  var $, Q, Z, B, O, J, G, L, C, V, D = W[0], E = W[1], H = W[2], I = Y[0], T = Y[1], R = Y[2], P = X[0], U = X[1], A = X[2];
  if (Math.abs(D - P) < N && Math.abs(E - U) < N && Math.abs(H - A) < N)
    return G0(K);
  if (G = D - P, L = E - U, C = H - A, V = 1 / Math.hypot(G, L, C), G *= V, L *= V, C *= V, $ = T * C - R * L, Q = R * G - I * C, Z = I * L - T * G, V = Math.hypot($, Q, Z), !V)
    $ = 0, Q = 0, Z = 0;
  else
    V = 1 / V, $ *= V, Q *= V, Z *= V;
  if (B = L * Z - C * Q, O = C * $ - G * Z, J = G * Q - L * $, V = Math.hypot(B, O, J), !V)
    B = 0, O = 0, J = 0;
  else
    V = 1 / V, B *= V, O *= V, J *= V;
  return K[0] = $, K[1] = B, K[2] = G, K[3] = 0, K[4] = Q, K[5] = O, K[6] = L, K[7] = 0, K[8] = Z, K[9] = J, K[10] = C, K[11] = 0, K[12] = -($ * D + Q * E + Z * H), K[13] = -(B * D + O * E + J * H), K[14] = -(G * D + L * E + C * H), K[15] = 1, K;
};
var nK = function(K, W, X, Y) {
  var $ = W[0], Q = W[1], Z = W[2], B = Y[0], O = Y[1], J = Y[2], G = $ - X[0], L = Q - X[1], C = Z - X[2], V = G * G + L * L + C * C;
  if (V > 0)
    V = 1 / Math.sqrt(V), G *= V, L *= V, C *= V;
  var D = O * C - J * L, E = J * G - B * C, H = B * L - O * G;
  if (V = D * D + E * E + H * H, V > 0)
    V = 1 / Math.sqrt(V), D *= V, E *= V, H *= V;
  return K[0] = D, K[1] = E, K[2] = H, K[3] = 0, K[4] = L * H - C * E, K[5] = C * D - G * H, K[6] = G * E - L * D, K[7] = 0, K[8] = G, K[9] = L, K[10] = C, K[11] = 0, K[12] = $, K[13] = Q, K[14] = Z, K[15] = 1, K;
};
var lK = function(K) {
  return "mat4(" + K[0] + ", " + K[1] + ", " + K[2] + ", " + K[3] + ", " + K[4] + ", " + K[5] + ", " + K[6] + ", " + K[7] + ", " + K[8] + ", " + K[9] + ", " + K[10] + ", " + K[11] + ", " + K[12] + ", " + K[13] + ", " + K[14] + ", " + K[15] + ")";
};
var cK = function(K) {
  return Math.hypot(K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8], K[9], K[10], K[11], K[12], K[13], K[14], K[15]);
};
var iK = function(K, W, X) {
  return K[0] = W[0] + X[0], K[1] = W[1] + X[1], K[2] = W[2] + X[2], K[3] = W[3] + X[3], K[4] = W[4] + X[4], K[5] = W[5] + X[5], K[6] = W[6] + X[6], K[7] = W[7] + X[7], K[8] = W[8] + X[8], K[9] = W[9] + X[9], K[10] = W[10] + X[10], K[11] = W[11] + X[11], K[12] = W[12] + X[12], K[13] = W[13] + X[13], K[14] = W[14] + X[14], K[15] = W[15] + X[15], K;
};
var D0 = function(K, W, X) {
  return K[0] = W[0] - X[0], K[1] = W[1] - X[1], K[2] = W[2] - X[2], K[3] = W[3] - X[3], K[4] = W[4] - X[4], K[5] = W[5] - X[5], K[6] = W[6] - X[6], K[7] = W[7] - X[7], K[8] = W[8] - X[8], K[9] = W[9] - X[9], K[10] = W[10] - X[10], K[11] = W[11] - X[11], K[12] = W[12] - X[12], K[13] = W[13] - X[13], K[14] = W[14] - X[14], K[15] = W[15] - X[15], K;
};
var yK = function(K, W, X) {
  return K[0] = W[0] * X, K[1] = W[1] * X, K[2] = W[2] * X, K[3] = W[3] * X, K[4] = W[4] * X, K[5] = W[5] * X, K[6] = W[6] * X, K[7] = W[7] * X, K[8] = W[8] * X, K[9] = W[9] * X, K[10] = W[10] * X, K[11] = W[11] * X, K[12] = W[12] * X, K[13] = W[13] * X, K[14] = W[14] * X, K[15] = W[15] * X, K;
};
var zK = function(K, W, X, Y) {
  return K[0] = W[0] + X[0] * Y, K[1] = W[1] + X[1] * Y, K[2] = W[2] + X[2] * Y, K[3] = W[3] + X[3] * Y, K[4] = W[4] + X[4] * Y, K[5] = W[5] + X[5] * Y, K[6] = W[6] + X[6] * Y, K[7] = W[7] + X[7] * Y, K[8] = W[8] + X[8] * Y, K[9] = W[9] + X[9] * Y, K[10] = W[10] + X[10] * Y, K[11] = W[11] + X[11] * Y, K[12] = W[12] + X[12] * Y, K[13] = W[13] + X[13] * Y, K[14] = W[14] + X[14] * Y, K[15] = W[15] + X[15] * Y, K;
};
var sK = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2] && K[3] === W[3] && K[4] === W[4] && K[5] === W[5] && K[6] === W[6] && K[7] === W[7] && K[8] === W[8] && K[9] === W[9] && K[10] === W[10] && K[11] === W[11] && K[12] === W[12] && K[13] === W[13] && K[14] === W[14] && K[15] === W[15];
};
var rK = function(K, W) {
  var X = K[0], Y = K[1], $ = K[2], Q = K[3], Z = K[4], B = K[5], O = K[6], J = K[7], G = K[8], L = K[9], C = K[10], V = K[11], D = K[12], E = K[13], H = K[14], I = K[15], T = W[0], R = W[1], P = W[2], U = W[3], A = W[4], p = W[5], h = W[6], j = W[7], M = W[8], F = W[9], q = W[10], g = W[11], S = W[12], f = W[13], i = W[14], y = W[15];
  return Math.abs(X - T) <= N * Math.max(1, Math.abs(X), Math.abs(T)) && Math.abs(Y - R) <= N * Math.max(1, Math.abs(Y), Math.abs(R)) && Math.abs($ - P) <= N * Math.max(1, Math.abs($), Math.abs(P)) && Math.abs(Q - U) <= N * Math.max(1, Math.abs(Q), Math.abs(U)) && Math.abs(Z - A) <= N * Math.max(1, Math.abs(Z), Math.abs(A)) && Math.abs(B - p) <= N * Math.max(1, Math.abs(B), Math.abs(p)) && Math.abs(O - h) <= N * Math.max(1, Math.abs(O), Math.abs(h)) && Math.abs(J - j) <= N * Math.max(1, Math.abs(J), Math.abs(j)) && Math.abs(G - M) <= N * Math.max(1, Math.abs(G), Math.abs(M)) && Math.abs(L - F) <= N * Math.max(1, Math.abs(L), Math.abs(F)) && Math.abs(C - q) <= N * Math.max(1, Math.abs(C), Math.abs(q)) && Math.abs(V - g) <= N * Math.max(1, Math.abs(V), Math.abs(g)) && Math.abs(D - S) <= N * Math.max(1, Math.abs(D), Math.abs(S)) && Math.abs(E - f) <= N * Math.max(1, Math.abs(E), Math.abs(f)) && Math.abs(H - i) <= N * Math.max(1, Math.abs(H), Math.abs(i)) && Math.abs(I - y) <= N * Math.max(1, Math.abs(I), Math.abs(y));
};
var e = function() {
  var K = new k(3);
  if (k != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0;
  return K;
};
var eK = function(K) {
  var W = new k(3);
  return W[0] = K[0], W[1] = K[1], W[2] = K[2], W;
};
var I0 = function(K) {
  var W = K[0], X = K[1], Y = K[2];
  return Math.hypot(W, X, Y);
};
var b = function(K, W, X) {
  var Y = new k(3);
  return Y[0] = K, Y[1] = W, Y[2] = X, Y;
};
var bK = function(K, W) {
  return K[0] = W[0], K[1] = W[1], K[2] = W[2], K;
};
var uK = function(K, W, X, Y) {
  return K[0] = W, K[1] = X, K[2] = Y, K;
};
var oK = function(K, W, X) {
  return K[0] = W[0] + X[0], K[1] = W[1] + X[1], K[2] = W[2] + X[2], K;
};
var U0 = function(K, W, X) {
  return K[0] = W[0] - X[0], K[1] = W[1] - X[1], K[2] = W[2] - X[2], K;
};
var A0 = function(K, W, X) {
  return K[0] = W[0] * X[0], K[1] = W[1] * X[1], K[2] = W[2] * X[2], K;
};
var P0 = function(K, W, X) {
  return K[0] = W[0] / X[0], K[1] = W[1] / X[1], K[2] = W[2] / X[2], K;
};
var tK = function(K, W) {
  return K[0] = Math.ceil(W[0]), K[1] = Math.ceil(W[1]), K[2] = Math.ceil(W[2]), K;
};
var aK = function(K, W) {
  return K[0] = Math.floor(W[0]), K[1] = Math.floor(W[1]), K[2] = Math.floor(W[2]), K;
};
var KW = function(K, W, X) {
  return K[0] = Math.min(W[0], X[0]), K[1] = Math.min(W[1], X[1]), K[2] = Math.min(W[2], X[2]), K;
};
var WW = function(K, W, X) {
  return K[0] = Math.max(W[0], X[0]), K[1] = Math.max(W[1], X[1]), K[2] = Math.max(W[2], X[2]), K;
};
var XW = function(K, W) {
  return K[0] = Math.round(W[0]), K[1] = Math.round(W[1]), K[2] = Math.round(W[2]), K;
};
var YW = function(K, W, X) {
  return K[0] = W[0] * X, K[1] = W[1] * X, K[2] = W[2] * X, K;
};
var $W = function(K, W, X, Y) {
  return K[0] = W[0] + X[0] * Y, K[1] = W[1] + X[1] * Y, K[2] = W[2] + X[2] * Y, K;
};
var R0 = function(K, W) {
  var X = W[0] - K[0], Y = W[1] - K[1], $ = W[2] - K[2];
  return Math.hypot(X, Y, $);
};
var N0 = function(K, W) {
  var X = W[0] - K[0], Y = W[1] - K[1], $ = W[2] - K[2];
  return X * X + Y * Y + $ * $;
};
var T0 = function(K) {
  var W = K[0], X = K[1], Y = K[2];
  return W * W + X * X + Y * Y;
};
var QW = function(K, W) {
  return K[0] = -W[0], K[1] = -W[1], K[2] = -W[2], K;
};
var ZW = function(K, W) {
  return K[0] = 1 / W[0], K[1] = 1 / W[1], K[2] = 1 / W[2], K;
};
var K0 = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q = X * X + Y * Y + $ * $;
  if (Q > 0)
    Q = 1 / Math.sqrt(Q);
  return K[0] = W[0] * Q, K[1] = W[1] * Q, K[2] = W[2] * Q, K;
};
var u = function(K, W) {
  return K[0] * W[0] + K[1] * W[1] + K[2] * W[2];
};
var s = function(K, W, X) {
  var Y = W[0], $ = W[1], Q = W[2], Z = X[0], B = X[1], O = X[2];
  return K[0] = $ * O - Q * B, K[1] = Q * Z - Y * O, K[2] = Y * B - $ * Z, K;
};
var BW = function(K, W, X, Y) {
  var $ = W[0], Q = W[1], Z = W[2];
  return K[0] = $ + Y * (X[0] - $), K[1] = Q + Y * (X[1] - Q), K[2] = Z + Y * (X[2] - Z), K;
};
var OW = function(K, W, X, Y, $, Q) {
  var Z = Q * Q, B = Z * (2 * Q - 3) + 1, O = Z * (Q - 2) + Q, J = Z * (Q - 1), G = Z * (3 - 2 * Q);
  return K[0] = W[0] * B + X[0] * O + Y[0] * J + $[0] * G, K[1] = W[1] * B + X[1] * O + Y[1] * J + $[1] * G, K[2] = W[2] * B + X[2] * O + Y[2] * J + $[2] * G, K;
};
var JW = function(K, W, X, Y, $, Q) {
  var Z = 1 - Q, B = Z * Z, O = Q * Q, J = B * Z, G = 3 * Q * B, L = 3 * O * Z, C = O * Q;
  return K[0] = W[0] * J + X[0] * G + Y[0] * L + $[0] * C, K[1] = W[1] * J + X[1] * G + Y[1] * L + $[1] * C, K[2] = W[2] * J + X[2] * G + Y[2] * L + $[2] * C, K;
};
var GW = function(K, W) {
  W = W || 1;
  var X = v() * 2 * Math.PI, Y = v() * 2 - 1, $ = Math.sqrt(1 - Y * Y) * W;
  return K[0] = Math.cos(X) * $, K[1] = Math.sin(X) * $, K[2] = Y * W, K;
};
var HW = function(K, W, X) {
  var Y = W[0], $ = W[1], Q = W[2], Z = X[3] * Y + X[7] * $ + X[11] * Q + X[15];
  return Z = Z || 1, K[0] = (X[0] * Y + X[4] * $ + X[8] * Q + X[12]) / Z, K[1] = (X[1] * Y + X[5] * $ + X[9] * Q + X[13]) / Z, K[2] = (X[2] * Y + X[6] * $ + X[10] * Q + X[14]) / Z, K;
};
var LW = function(K, W, X) {
  var Y = W[0], $ = W[1], Q = W[2];
  return K[0] = Y * X[0] + $ * X[3] + Q * X[6], K[1] = Y * X[1] + $ * X[4] + Q * X[7], K[2] = Y * X[2] + $ * X[5] + Q * X[8], K;
};
var VW = function(K, W, X) {
  var Y = X[0], $ = X[1], Q = X[2], Z = X[3], B = W[0], O = W[1], J = W[2], G = $ * J - Q * O, L = Q * B - Y * J, C = Y * O - $ * B, V = $ * C - Q * L, D = Q * G - Y * C, E = Y * L - $ * G, H = Z * 2;
  return G *= H, L *= H, C *= H, V *= 2, D *= 2, E *= 2, K[0] = B + G + V, K[1] = O + L + D, K[2] = J + C + E, K;
};
var CW = function(K, W, X, Y) {
  var $ = [], Q = [];
  return $[0] = W[0] - X[0], $[1] = W[1] - X[1], $[2] = W[2] - X[2], Q[0] = $[0], Q[1] = $[1] * Math.cos(Y) - $[2] * Math.sin(Y), Q[2] = $[1] * Math.sin(Y) + $[2] * Math.cos(Y), K[0] = Q[0] + X[0], K[1] = Q[1] + X[1], K[2] = Q[2] + X[2], K;
};
var EW = function(K, W, X, Y) {
  var $ = [], Q = [];
  return $[0] = W[0] - X[0], $[1] = W[1] - X[1], $[2] = W[2] - X[2], Q[0] = $[2] * Math.sin(Y) + $[0] * Math.cos(Y), Q[1] = $[1], Q[2] = $[2] * Math.cos(Y) - $[0] * Math.sin(Y), K[0] = Q[0] + X[0], K[1] = Q[1] + X[1], K[2] = Q[2] + X[2], K;
};
var DW = function(K, W, X, Y) {
  var $ = [], Q = [];
  return $[0] = W[0] - X[0], $[1] = W[1] - X[1], $[2] = W[2] - X[2], Q[0] = $[0] * Math.cos(Y) - $[1] * Math.sin(Y), Q[1] = $[0] * Math.sin(Y) + $[1] * Math.cos(Y), Q[2] = $[2], K[0] = Q[0] + X[0], K[1] = Q[1] + X[1], K[2] = Q[2] + X[2], K;
};
var IW = function(K, W) {
  var X = K[0], Y = K[1], $ = K[2], Q = W[0], Z = W[1], B = W[2], O = Math.sqrt(X * X + Y * Y + $ * $), J = Math.sqrt(Q * Q + Z * Z + B * B), G = O * J, L = G && u(K, W) / G;
  return Math.acos(Math.min(Math.max(L, -1), 1));
};
var UW = function(K) {
  return K[0] = 0, K[1] = 0, K[2] = 0, K;
};
var AW = function(K) {
  return "vec3(" + K[0] + ", " + K[1] + ", " + K[2] + ")";
};
var PW = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2];
};
var RW = function(K, W) {
  var X = K[0], Y = K[1], $ = K[2], Q = W[0], Z = W[1], B = W[2];
  return Math.abs(X - Q) <= N * Math.max(1, Math.abs(X), Math.abs(Q)) && Math.abs(Y - Z) <= N * Math.max(1, Math.abs(Y), Math.abs(Z)) && Math.abs($ - B) <= N * Math.max(1, Math.abs($), Math.abs(B));
};
var MW = function() {
  var K = new k(4);
  if (k != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0, K[3] = 0;
  return K;
};
var S0 = function(K) {
  var W = new k(4);
  return W[0] = K[0], W[1] = K[1], W[2] = K[2], W[3] = K[3], W;
};
var _0 = function(K, W, X, Y) {
  var $ = new k(4);
  return $[0] = K, $[1] = W, $[2] = X, $[3] = Y, $;
};
var k0 = function(K, W) {
  return K[0] = W[0], K[1] = W[1], K[2] = W[2], K[3] = W[3], K;
};
var h0 = function(K, W, X, Y, $) {
  return K[0] = W, K[1] = X, K[2] = Y, K[3] = $, K;
};
var j0 = function(K, W, X) {
  return K[0] = W[0] + X[0], K[1] = W[1] + X[1], K[2] = W[2] + X[2], K[3] = W[3] + X[3], K;
};
var M0 = function(K, W, X) {
  return K[0] = W[0] * X, K[1] = W[1] * X, K[2] = W[2] * X, K[3] = W[3] * X, K;
};
var p0 = function(K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3];
  return Math.hypot(W, X, Y, $);
};
var F0 = function(K) {
  var W = K[0], X = K[1], Y = K[2], $ = K[3];
  return W * W + X * X + Y * Y + $ * $;
};
var q0 = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q = W[3], Z = X * X + Y * Y + $ * $ + Q * Q;
  if (Z > 0)
    Z = 1 / Math.sqrt(Z);
  return K[0] = X * Z, K[1] = Y * Z, K[2] = $ * Z, K[3] = Q * Z, K;
};
var g0 = function(K, W) {
  return K[0] * W[0] + K[1] * W[1] + K[2] * W[2] + K[3] * W[3];
};
var f0 = function(K, W, X, Y) {
  var $ = W[0], Q = W[1], Z = W[2], B = W[3];
  return K[0] = $ + Y * (X[0] - $), K[1] = Q + Y * (X[1] - Q), K[2] = Z + Y * (X[2] - Z), K[3] = B + Y * (X[3] - B), K;
};
var w0 = function(K, W) {
  return K[0] === W[0] && K[1] === W[1] && K[2] === W[2] && K[3] === W[3];
};
var d0 = function(K, W) {
  var X = K[0], Y = K[1], $ = K[2], Q = K[3], Z = W[0], B = W[1], O = W[2], J = W[3];
  return Math.abs(X - Z) <= N * Math.max(1, Math.abs(X), Math.abs(Z)) && Math.abs(Y - B) <= N * Math.max(1, Math.abs(Y), Math.abs(B)) && Math.abs($ - O) <= N * Math.max(1, Math.abs($), Math.abs(O)) && Math.abs(Q - J) <= N * Math.max(1, Math.abs(Q), Math.abs(J));
};
var X0 = function() {
  var K = new k(4);
  if (k != Float32Array)
    K[0] = 0, K[1] = 0, K[2] = 0;
  return K[3] = 1, K;
};
var FW = function(K) {
  return K[0] = 0, K[1] = 0, K[2] = 0, K[3] = 1, K;
};
var v0 = function(K, W, X) {
  X = X * 0.5;
  var Y = Math.sin(X);
  return K[0] = Y * W[0], K[1] = Y * W[1], K[2] = Y * W[2], K[3] = Math.cos(X), K;
};
var qW = function(K, W) {
  var X = Math.acos(W[3]) * 2, Y = Math.sin(X / 2);
  if (Y > N)
    K[0] = W[0] / Y, K[1] = W[1] / Y, K[2] = W[2] / Y;
  else
    K[0] = 1, K[1] = 0, K[2] = 0;
  return X;
};
var gW = function(K, W) {
  var X = z0(K, W);
  return Math.acos(2 * X * X - 1);
};
var n0 = function(K, W, X) {
  var Y = W[0], $ = W[1], Q = W[2], Z = W[3], B = X[0], O = X[1], J = X[2], G = X[3];
  return K[0] = Y * G + Z * B + $ * J - Q * O, K[1] = $ * G + Z * O + Q * B - Y * J, K[2] = Q * G + Z * J + Y * O - $ * B, K[3] = Z * G - Y * B - $ * O - Q * J, K;
};
var fW = function(K, W, X) {
  X *= 0.5;
  var Y = W[0], $ = W[1], Q = W[2], Z = W[3], B = Math.sin(X), O = Math.cos(X);
  return K[0] = Y * O + Z * B, K[1] = $ * O + Q * B, K[2] = Q * O - $ * B, K[3] = Z * O - Y * B, K;
};
var wW = function(K, W, X) {
  X *= 0.5;
  var Y = W[0], $ = W[1], Q = W[2], Z = W[3], B = Math.sin(X), O = Math.cos(X);
  return K[0] = Y * O - Q * B, K[1] = $ * O + Z * B, K[2] = Q * O + Y * B, K[3] = Z * O - $ * B, K;
};
var dW = function(K, W, X) {
  X *= 0.5;
  var Y = W[0], $ = W[1], Q = W[2], Z = W[3], B = Math.sin(X), O = Math.cos(X);
  return K[0] = Y * O + $ * B, K[1] = $ * O - Y * B, K[2] = Q * O + Z * B, K[3] = Z * O - Q * B, K;
};
var vW = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2];
  return K[0] = X, K[1] = Y, K[2] = $, K[3] = Math.sqrt(Math.abs(1 - X * X - Y * Y - $ * $)), K;
};
var l0 = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q = W[3], Z = Math.sqrt(X * X + Y * Y + $ * $), B = Math.exp(Q), O = Z > 0 ? B * Math.sin(Z) / Z : 0;
  return K[0] = X * O, K[1] = Y * O, K[2] = $ * O, K[3] = B * Math.cos(Z), K;
};
var c0 = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q = W[3], Z = Math.sqrt(X * X + Y * Y + $ * $), B = Z > 0 ? Math.atan2(Z, Q) / Z : 0;
  return K[0] = X * B, K[1] = Y * B, K[2] = $ * B, K[3] = 0.5 * Math.log(X * X + Y * Y + $ * $ + Q * Q), K;
};
var nW = function(K, W, X) {
  return c0(K, W), y0(K, K, X), l0(K, K), K;
};
var o = function(K, W, X, Y) {
  var $ = W[0], Q = W[1], Z = W[2], B = W[3], O = X[0], J = X[1], G = X[2], L = X[3], C, V, D, E, H;
  if (V = $ * O + Q * J + Z * G + B * L, V < 0)
    V = -V, O = -O, J = -J, G = -G, L = -L;
  if (1 - V > N)
    C = Math.acos(V), D = Math.sin(C), E = Math.sin((1 - Y) * C) / D, H = Math.sin(Y * C) / D;
  else
    E = 1 - Y, H = Y;
  return K[0] = E * $ + H * O, K[1] = E * Q + H * J, K[2] = E * Z + H * G, K[3] = E * B + H * L, K;
};
var lW = function(K) {
  var W = v(), X = v(), Y = v(), $ = Math.sqrt(1 - W), Q = Math.sqrt(W);
  return K[0] = $ * Math.sin(2 * Math.PI * X), K[1] = $ * Math.cos(2 * Math.PI * X), K[2] = Q * Math.sin(2 * Math.PI * Y), K[3] = Q * Math.cos(2 * Math.PI * Y), K;
};
var cW = function(K, W) {
  var X = W[0], Y = W[1], $ = W[2], Q = W[3], Z = X * X + Y * Y + $ * $ + Q * Q, B = Z ? 1 / Z : 0;
  return K[0] = -X * B, K[1] = -Y * B, K[2] = -$ * B, K[3] = Q * B, K;
};
var iW = function(K, W) {
  return K[0] = -W[0], K[1] = -W[1], K[2] = -W[2], K[3] = W[3], K;
};
var i0 = function(K, W) {
  var X = W[0] + W[4] + W[8], Y;
  if (X > 0)
    Y = Math.sqrt(X + 1), K[3] = 0.5 * Y, Y = 0.5 / Y, K[0] = (W[5] - W[7]) * Y, K[1] = (W[6] - W[2]) * Y, K[2] = (W[1] - W[3]) * Y;
  else {
    var $ = 0;
    if (W[4] > W[0])
      $ = 1;
    if (W[8] > W[$ * 3 + $])
      $ = 2;
    var Q = ($ + 1) % 3, Z = ($ + 2) % 3;
    Y = Math.sqrt(W[$ * 3 + $] - W[Q * 3 + Q] - W[Z * 3 + Z] + 1), K[$] = 0.5 * Y, Y = 0.5 / Y, K[3] = (W[Q * 3 + Z] - W[Z * 3 + Q]) * Y, K[Q] = (W[Q * 3 + $] + W[$ * 3 + Q]) * Y, K[Z] = (W[Z * 3 + $] + W[$ * 3 + Z]) * Y;
  }
  return K;
};
var yW = function(K, W, X, Y) {
  var $ = 0.5 * Math.PI / 180;
  W *= $, X *= $, Y *= $;
  var Q = Math.sin(W), Z = Math.cos(W), B = Math.sin(X), O = Math.cos(X), J = Math.sin(Y), G = Math.cos(Y);
  return K[0] = Q * O * G - Z * B * J, K[1] = Z * B * G + Q * O * J, K[2] = Z * O * J - Q * B * G, K[3] = Z * O * G + Q * B * J, K;
};
var zW = function(K) {
  return "quat(" + K[0] + ", " + K[1] + ", " + K[2] + ", " + K[3] + ")";
};
var WK = Object.defineProperty;
var a = (K, W) => {
  for (var X in W)
    WK(K, X, { get: W[X], enumerable: true, configurable: true, set: (Y) => W[X] = () => Y });
};
var N = 0.000001;
var k = typeof Float32Array !== "undefined" ? Float32Array : Array;
var v = Math.random;
var LX = Math.PI / 180;
if (!Math.hypot)
  Math.hypot = function() {
    var K = 0, W = arguments.length;
    while (W--)
      K += arguments[W] * arguments[W];
    return Math.sqrt(K);
  };
var _ = {};
a(_, { transpose: () => {
  {
    return JK;
  }
}, translate: () => {
  {
    return VK;
  }
}, targetTo: () => {
  {
    return nK;
  }
}, subtract: () => {
  {
    return D0;
  }
}, sub: () => {
  {
    return xK;
  }
}, str: () => {
  {
    return lK;
  }
}, set: () => {
  {
    return OK;
  }
}, scale: () => {
  {
    return CK;
  }
}, rotateZ: () => {
  {
    return UK;
  }
}, rotateY: () => {
  {
    return IK;
  }
}, rotateX: () => {
  {
    return DK;
  }
}, rotate: () => {
  {
    return EK;
  }
}, perspectiveZO: () => {
  {
    return gK;
  }
}, perspectiveNO: () => {
  {
    return C0;
  }
}, perspectiveFromFieldOfView: () => {
  {
    return fK;
  }
}, perspective: () => {
  {
    return qK;
  }
}, orthoZO: () => {
  {
    return dK;
  }
}, orthoNO: () => {
  {
    return E0;
  }
}, ortho: () => {
  {
    return wK;
  }
}, multiplyScalarAndAdd: () => {
  {
    return zK;
  }
}, multiplyScalar: () => {
  {
    return yK;
  }
}, multiply: () => {
  {
    return H0;
  }
}, mul: () => {
  {
    return mK;
  }
}, lookAt: () => {
  {
    return vK;
  }
}, invert: () => {
  {
    return GK;
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
    return V0;
  }
}, getRotation: () => {
  {
    return hK;
  }
}, frustum: () => {
  {
    return FK;
  }
}, fromZRotation: () => {
  {
    return SK;
  }
}, fromYRotation: () => {
  {
    return TK;
  }
}, fromXRotation: () => {
  {
    return NK;
  }
}, fromValues: () => {
  {
    return BK;
  }
}, fromTranslation: () => {
  {
    return AK;
  }
}, fromScaling: () => {
  {
    return PK;
  }
}, fromRotationTranslationScaleOrigin: () => {
  {
    return MK;
  }
}, fromRotationTranslationScale: () => {
  {
    return jK;
  }
}, fromRotationTranslation: () => {
  {
    return L0;
  }
}, fromRotation: () => {
  {
    return RK;
  }
}, fromQuat2: () => {
  {
    return _K;
  }
}, fromQuat: () => {
  {
    return pK;
  }
}, frob: () => {
  {
    return cK;
  }
}, exactEquals: () => {
  {
    return sK;
  }
}, equals: () => {
  {
    return rK;
  }
}, determinant: () => {
  {
    return LK;
  }
}, create: () => {
  {
    return $K;
  }
}, copy: () => {
  {
    return ZK;
  }
}, clone: () => {
  {
    return QK;
  }
}, adjoint: () => {
  {
    return HK;
  }
}, add: () => {
  {
    return iK;
  }
} });
var qK = C0;
var wK = E0;
var mK = H0;
var xK = D0;
var m = {};
a(m, { str: () => {
  {
    return zW;
  }
}, squaredLength: () => {
  {
    return r0;
  }
}, sqrLen: () => {
  {
    return tW;
  }
}, sqlerp: () => {
  {
    return XX;
  }
}, slerp: () => {
  {
    return o;
  }
}, setAxisAngle: () => {
  {
    return v0;
  }
}, setAxes: () => {
  {
    return YX;
  }
}, set: () => {
  {
    return xW;
  }
}, scale: () => {
  {
    return y0;
  }
}, rotationTo: () => {
  {
    return WX;
  }
}, rotateZ: () => {
  {
    return dW;
  }
}, rotateY: () => {
  {
    return wW;
  }
}, rotateX: () => {
  {
    return fW;
  }
}, random: () => {
  {
    return lW;
  }
}, pow: () => {
  {
    return nW;
  }
}, normalize: () => {
  {
    return Y0;
  }
}, multiply: () => {
  {
    return n0;
  }
}, mul: () => {
  {
    return bW;
  }
}, ln: () => {
  {
    return c0;
  }
}, lerp: () => {
  {
    return uW;
  }
}, length: () => {
  {
    return s0;
  }
}, len: () => {
  {
    return oW;
  }
}, invert: () => {
  {
    return cW;
  }
}, identity: () => {
  {
    return FW;
  }
}, getAxisAngle: () => {
  {
    return qW;
  }
}, getAngle: () => {
  {
    return gW;
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
    return yW;
  }
}, exp: () => {
  {
    return l0;
  }
}, exactEquals: () => {
  {
    return aW;
  }
}, equals: () => {
  {
    return KX;
  }
}, dot: () => {
  {
    return z0;
  }
}, create: () => {
  {
    return X0;
  }
}, copy: () => {
  {
    return mW;
  }
}, conjugate: () => {
  {
    return iW;
  }
}, clone: () => {
  {
    return sW;
  }
}, calculateW: () => {
  {
    return vW;
  }
}, add: () => {
  {
    return eW;
  }
} });
var r = {};
a(r, { zero: () => {
  {
    return UW;
  }
}, transformQuat: () => {
  {
    return VW;
  }
}, transformMat4: () => {
  {
    return HW;
  }
}, transformMat3: () => {
  {
    return LW;
  }
}, subtract: () => {
  {
    return U0;
  }
}, sub: () => {
  {
    return NW;
  }
}, str: () => {
  {
    return AW;
  }
}, squaredLength: () => {
  {
    return T0;
  }
}, squaredDistance: () => {
  {
    return N0;
  }
}, sqrLen: () => {
  {
    return hW;
  }
}, sqrDist: () => {
  {
    return kW;
  }
}, set: () => {
  {
    return uK;
  }
}, scaleAndAdd: () => {
  {
    return $W;
  }
}, scale: () => {
  {
    return YW;
  }
}, round: () => {
  {
    return XW;
  }
}, rotateZ: () => {
  {
    return DW;
  }
}, rotateY: () => {
  {
    return EW;
  }
}, rotateX: () => {
  {
    return CW;
  }
}, random: () => {
  {
    return GW;
  }
}, normalize: () => {
  {
    return K0;
  }
}, negate: () => {
  {
    return QW;
  }
}, multiply: () => {
  {
    return A0;
  }
}, mul: () => {
  {
    return TW;
  }
}, min: () => {
  {
    return KW;
  }
}, max: () => {
  {
    return WW;
  }
}, lerp: () => {
  {
    return BW;
  }
}, length: () => {
  {
    return I0;
  }
}, len: () => {
  {
    return W0;
  }
}, inverse: () => {
  {
    return ZW;
  }
}, hermite: () => {
  {
    return OW;
  }
}, fromValues: () => {
  {
    return b;
  }
}, forEach: () => {
  {
    return jW;
  }
}, floor: () => {
  {
    return aK;
  }
}, exactEquals: () => {
  {
    return PW;
  }
}, equals: () => {
  {
    return RW;
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
    return SW;
  }
}, distance: () => {
  {
    return R0;
  }
}, dist: () => {
  {
    return _W;
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
    return bK;
  }
}, clone: () => {
  {
    return eK;
  }
}, ceil: () => {
  {
    return tK;
  }
}, bezier: () => {
  {
    return JW;
  }
}, angle: () => {
  {
    return IW;
  }
}, add: () => {
  {
    return oK;
  }
} });
var NW = U0;
var TW = A0;
var SW = P0;
var _W = R0;
var kW = N0;
var W0 = I0;
var hW = T0;
var jW = function() {
  var K = e();
  return function(W, X, Y, $, Q, Z) {
    var B, O;
    if (!X)
      X = 3;
    if (!Y)
      Y = 0;
    if ($)
      O = Math.min($ * X + Y, W.length);
    else
      O = W.length;
    for (B = Y;B < O; B += X)
      K[0] = W[B], K[1] = W[B + 1], K[2] = W[B + 2], Q(K, K, Z), W[B] = K[0], W[B + 1] = K[1], W[B + 2] = K[2];
    return W;
  };
}();
var VX = function() {
  var K = MW();
  return function(W, X, Y, $, Q, Z) {
    var B, O;
    if (!X)
      X = 4;
    if (!Y)
      Y = 0;
    if ($)
      O = Math.min($ * X + Y, W.length);
    else
      O = W.length;
    for (B = Y;B < O; B += X)
      K[0] = W[B], K[1] = W[B + 1], K[2] = W[B + 2], K[3] = W[B + 3], Q(K, K, Z), W[B] = K[0], W[B + 1] = K[1], W[B + 2] = K[2], W[B + 3] = K[3];
    return W;
  };
}();
var sW = S0;
var rW = _0;
var mW = k0;
var xW = h0;
var eW = j0;
var bW = n0;
var y0 = M0;
var z0 = g0;
var uW = f0;
var s0 = p0;
var oW = s0;
var r0 = F0;
var tW = r0;
var Y0 = q0;
var aW = w0;
var KX = d0;
var WX = function() {
  var K = e(), W = b(1, 0, 0), X = b(0, 1, 0);
  return function(Y, $, Q) {
    var Z = u($, Q);
    if (Z < -0.999999) {
      if (s(K, W, $), W0(K) < 0.000001)
        s(K, X, $);
      return K0(K, K), v0(Y, K, Math.PI), Y;
    } else if (Z > 0.999999)
      return Y[0] = 0, Y[1] = 0, Y[2] = 0, Y[3] = 1, Y;
    else
      return s(K, $, Q), Y[0] = K[0], Y[1] = K[1], Y[2] = K[2], Y[3] = 1 + Z, Y0(Y, Y);
  };
}();
var XX = function() {
  var K = X0(), W = X0();
  return function(X, Y, $, Q, Z, B) {
    return o(K, Y, Z, B), o(W, $, Q, B), o(X, K, W, 2 * B * (1 - B)), X;
  };
}();
var YX = function() {
  var K = J0();
  return function(W, X, Y, $) {
    return K[0] = Y[0], K[3] = Y[1], K[6] = Y[2], K[1] = $[0], K[4] = $[1], K[7] = $[2], K[2] = -X[0], K[5] = -X[1], K[8] = -X[2], Y0(W, i0(W, K));
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
var $X = Math.PI / 90;
var $0 = [0, 0, 0];
var m0 = _.create();
var x0 = _.create();
var t = m.create();

class x {
  static HIDDEN = x.create().scale(0, 0, 0);
  static IDENTITY = x.create();
  #K = Float32Array.from(_.create());
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
    return _.copy(this.#K, K.getMatrix()), this.#W.onChange(), this;
  }
  identity() {
    return _.identity(this.#K), this.#W.onChange(), this;
  }
  invert(K) {
    return _.invert(this.#K, K?.getMatrix() ?? this.getMatrix()), this.#W.onChange(), this;
  }
  multiply(K) {
    return _.multiply(this.#K, this.#K, K.getMatrix()), this.#W.onChange(), this;
  }
  multiply2(K, W) {
    return _.multiply(this.#K, K.getMatrix(), W.getMatrix()), this.#W.onChange(), this;
  }
  multiply3(K, W, X) {
    return this.multiply2(K, W), this.multiply(X), this;
  }
  translate(K, W, X) {
    const Y = $0;
    return Y[0] = K, Y[1] = W, Y[2] = X, this.move(Y);
  }
  move(K) {
    return _.translate(this.#K, this.#K, K), this.#W.onChange(), this;
  }
  rotateX(K) {
    return _.rotateX(this.#K, this.#K, K), this.#W.onChange(), this;
  }
  rotateY(K) {
    return _.rotateY(this.#K, this.#K, K), this.#W.onChange(), this;
  }
  rotateZ(K) {
    return _.rotateZ(this.#K, this.#K, K), this.#W.onChange(), this;
  }
  setXRotation(K) {
    return _.fromXRotation(this.getMatrix(), K), this.#W.onChange(), this;
  }
  setYRotation(K) {
    return _.fromYRotation(this.getMatrix(), K), this.#W.onChange(), this;
  }
  scale(K, W, X) {
    return _.scale(this.#K, this.#K, [K, W ?? K, X ?? K]), this.#W.onChange(), this;
  }
  perspective(K, W, X, Y) {
    return _.perspective(this.#K, K * $X, W, X, Y), this.#W.onChange(), this;
  }
  ortho(K, W, X, Y, $, Q) {
    return _.ortho(this.#K, K, W, X, Y, $, Q), this.#W.onChange(), this;
  }
  combine(K, W, X = 0.5) {
    return _.multiplyScalar(m0, K.getMatrix(), 1 - X), _.multiplyScalar(x0, W.getMatrix(), X), _.add(this.#K, m0, x0), this.#W.onChange(), this;
  }
  static getMoveVector(K, W, X, Y) {
    const $ = $0;
    if ($[0] = K, $[1] = W, $[2] = X, Y)
      _.getRotation(t, Y.getMatrix()), m.invert(t, t), r.transformQuat($, $, t);
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

class e0 {
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

class b0 {
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

class u0 extends b0 {
  constructor() {
    super((K, W) => {
      if (!K)
        return new e0(W, (X) => X.valueOf(), (X, Y) => X.setValue(Y));
      return K.element = W, K;
    });
  }
}
var QX = new u0;

class l {
  w;
  q;
  #K = 0;
  #W;
  constructor(K = 0, W, X = QX) {
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
class t0 {
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

class Q0 extends t0 {
  constructor() {
    super((K, W, X, Y) => {
      if (!K)
        return [W, X, Y];
      return K[0] = W, K[1] = X, K[2] = Y, K;
    });
  }
}

class c {
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
  toVector(K, W, X) {
    return this.motor.scheduleUpdate(this, this.data), this.vectorPool.create(K, W, X);
  }
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
var Z0 = function(K, W) {
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

class a0 {
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
    const $ = w.getMoveVector(K, W, X, Y), Q = Z0(this.blockers, (Z) => Z.isBlocked(c.toVector(this.position[0] + $[0], this.position[1] + $[1], this.position[2] + $[2], this.#W), this.position));
    if (!Q)
      if ($[0] || $[1] || $[2])
        this.#K.move($);
      else
        return d.AT_POSITION;
    return Q ? d.BLOCKED : d.MOVED;
  }
  moveTo(K, W, X) {
    if (this.position[0] === K && this.position[1] === W && this.position[2] === X)
      return d.AT_POSITION;
    const Y = Z0(this.blockers, ($) => $.isBlocked(c.toVector(K, W, X, this.#W), this.position));
    if (!Y) {
      const [$, Q, Z] = this.#K.getPosition();
      if ($ !== K || Q !== W || Z !== X)
        this.#K.setPosition(K, W, X);
    }
    return Y ? d.BLOCKED : d.MOVED;
  }
  movedTo(K, W, X) {
    return this.moveTo(K, W, X), this;
  }
  gotoPos(K, W, X, Y = 0.1) {
    const $ = this.position, Q = K - $[0], Z = W - $[1], B = X - $[2], O = Math.sqrt(Q * Q + Z * Z + B * B);
    if (O > 0.01) {
      const J = Math.min(O, Y);
      return this.moveBy(Q / O * J, Z / O * J, B / O * J);
    } else
      return this.moveTo(K, W, X);
  }
  getMatrix() {
    return this.#K.getMatrix();
  }
}
var OX = 1;
var JX = 1;

class KK {
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
    this.perspective = new l(OX, K.onChange), this.zoom = new l(JX, (W) => {
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
    const $ = this.#Y[0] / this.#Y[1], Q = 45 / Math.sqrt(W);
    this.configPerspectiveMatrix(Q, $, Math.max(X, 0.00001), Y), this.configOrthoMatrix($ / W / W, 1 / W / W, -Y, Y);
  }
  getMatrix() {
    return this.#K.getMatrix();
  }
}
export {
  w as Matrix
};
