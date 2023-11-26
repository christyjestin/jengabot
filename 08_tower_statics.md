# 08 - Tower Statics

We index each block using three numbers, $i$, $j$, and $k$, which index the $x$, $y$, and $z$ coordinates of the block.  The mass of each block is $m_{ijk}$, and the total mass of the tower is $M$. We assume that the tower is in static equilibrium, so the net force on the tower is zero. We also assume that each block has the same mass, i.e. $m_{ijk} = m$ $\forall$ $i$, $j$, and $k$. 

Because the Jenga block begins as 18 blocks high, and three wide, we have the following initial conditions. Let $\mathcal{N}$ be the set that uniquely indexes each block. Let $\mathcal{K}_t$ be the set of admissiable $k$ values at time $t$. Similarly, let $\mathcal{I}$ and $\mathcal{J}$ be the set of admissiable $i$ and $j$ values. Then, we have the following initial conditions:

$$
\begin{align}
    \mathcal{K}_0 &= \{1,\dotsc,18\} \\
    \mathcal{I} &= \{1,\dotsc,3\} \\
    \mathcal{J} &= \{1,\dotsc,3\}. \\
    \mathcal{N} &=  \{1,\dotsc,\}\\
\end{align}
$$

Not that only $\mathcal{K}$ is a function of time. The other sets are constant. At time $t=1$, the $\max{(\mathcal{K})} = 19$ because the block is removed from the tower and placed on top of the tower. To gain an intuition, consider the following evolution of $\mathcal{K}$:

$$
\begin{aligned}
    \mathcal{K}_0 &= \{1,\dotsc,18\} \\
    \mathcal{K}_1 &= \{1,\dotsc,19\} \\
    \mathcal{K}_2 &= \{1,\dotsc,19\} \\
    \mathcal{K}_3 &= \{1,\dotsc,19\} \\
    \mathcal{K}_4 &= \{1,\dotsc,20\} \\
    \vdots \\
    \mathcal{K}_t &= \{1,\dotsc,18 + \mathrm{ceil}(t/3)\} \\
\end{aligned}
$$

The total number of blocks is $n = 18\times3 = 54$. The total mass of the tower is $M = n m$. The center of mass of the tower is $(x_{cm}, y_{cm}, z_{cm})$. The center of mass of the tower is given by

$$
\begin{align}
    M &= \sum_{i=1}^{n} m_i \\
    x_{cm} &= \frac{1}{M} \sum_{i=1}^{n} m_i x_i \\
    y_{cm} &= \frac{1}{M} \sum_{i=1}^{n} m_i y_i \\
    z_{cm} &= \frac{1}{M} \sum_{i=1}^{n} m_i z_i \\
\end{align}
$$

