function dy = cartpend(y,m,M,L,g,d,u)

Sy = sin(y(3));
Cy = cos(y(3));
den = M + m * Sy^2;

dy(1, 1) = y(2);
dy(2, 1) = (m * Sy * (L * y(4)^2 - g * Cy) - d * y(2) + u) / den;
dy(3, 1) = y(4);
dy(4, 1) = ((M + m) * g * Sy - m * L * y(4)^2 * Sy * Cy + d * y(2)* Cy + Cy * u)/(L * den);