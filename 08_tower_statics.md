# 08 - Tower Statics

## Rigid-body *statics without* block friction

Static equilibrium (absence external disturbances) is the most fundamental property that the tower must have in order for it to not topple. To start, masses and inertias of the blocks will be used to compute the net wrench
(force and moment) due to gravity, without considering frictional effects or minor deviations in the blocks’ nominal dimensions.

We index each block using three numbers, $i$, $j$, and $k$, which index the $x$, $y$, and $z$ coordinates of the block.  The mass of each block is $m_{ijk}$, and the total mass of the tower is $M$. We assume that the tower is in static equilibrium, so the net force on the tower is zero. We also assume that each block has the same mass, i.e. $m_{ijk} = m$ $\forall$ $i$, $j$, and $k$.

Below is a sketch of the tower.

![Tower Sketch](./assets/jenga_sketch.jpg)

Because the Jenga tower begins as 18 blocks high, and three wide, we can define the following sets to help us identify the blocks in e. Let $\mathcal{N}$ be the set that uniquely indexes each block, and $\max {\mathcal{N}}=n=18 \times 3$  be the total number of blocks. Let $\mathcal{K}_t$ be the set of admissiable $k$ values at time $t$. Similarly, let $\mathcal{I}$ and $\mathcal{J}$ be the set of admissiable $i$ and $j$ values. Then, we have the following initial conditions:

$$
\begin{align}
    \mathcal{K}_0 &= \{1,\dotsc,18\} \\
    \mathcal{I} &= \{1,\dotsc,3\} \\
    \mathcal{J} &= \{1,\dotsc,3\}. \\
    \mathcal{N} &=  \{1,\dotsc,54\}\\
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

Note that we assume that the blocks are removed such that the tower never "jumps" downard. That is, we assume that a block is removed from the tower only if there is another block in its layer.

The center of mass of the tower is $(x_{cm}, y_{cm}, z_{cm})$. The center of mass of the tower is given by

$$
\begin{align}
    M &= \sum_{i=1}^{n} m_i \\
    x_{cm} &= \frac{1}{M} \sum_{i=1}^{n} m_i x_i \\
    y_{cm} &= \frac{1}{M} \sum_{i=1}^{n} m_i y_i \\
    z_{cm} &= \frac{1}{M} \sum_{i=1}^{n} m_i z_i \\
\end{align}
$$

where $x_i$, $y_i$, and $z_i$ are the $x$, $y$, and $z$ coordinates of block $i$. 

The center of mass of the tower is a function of time, because the mass of the tower changes as blocks are removed. The center of mass of the tower is also a function of the position of each block, which is a function of time. 

Now, we will study, for a given block $n \in \mathcal{N}$, its position in terms of $i \in \mathcal{I}$, $j \in \mathcal{J}$, and $k \in \mathcal{K}$. At locations

$$
\begin{align}
    p_{CM}(1,1,1) &= [l/2,\, w/2,\, h/2]^\top \\
    p_{CM}(1,1,2) &= [w/2,\, l/2,\, 3h/2]^\top \\
    p_{CM}(1,1,3) &= [l/2,\, w/2,\, 5h/2]^\top \\
    &\vdots \\
    p_{CM}(i,j,k)  &= 
    \begin{cases}
    \frac{1}{2}[(2i-1)l,\, (2j-1)w,\, (2k-1)h]^\top & \mathrm{rem}(k/2)=1\\
    \frac{1}{2}[(2i-1)w,\, (2j-1)l,\, (2k-1)h]^\top & \mathrm{rem}(k/2)=0\\
    \end{cases}
\end{align}
$$

Note that because the orientation of the stacked blocks alternates on each layer, we must use the conditional statement based on the oddness and eveness of $k$.

In order for the tower to not topple, its resultant center of mass above every layer must be inside the area polygon formed by the edges corners of the layer.

Below is some pseudocode for the algorithm that determines whether the tower will topple.

## *Algorithm 1*: Determine Feasible Blocks to Remove on a Given Turn
1. **Input**: Block mass $m$, width $w$, length $l$, height $h$, and number of blocks $n$.
2. **Output** Array of booleans $\mathcal{B}$, where $\mathcal{B}[i]$ is $\texttt{True}$ if the tower will topple if block $i$ is removed, and $\texttt{False}$ otherwise.
3. **for** each block $d\in \mathcal{N}$
   1. $(a,b,c) \leftarrow (i,j,k)$ position of the block $d$
   2. **for** each $k \in \mathcal{K}$, $k>c$
      1. Compute $x_{cm}$, $y_{cm}$, and $z_{cm}$ of the block.
      2. $p_d := (x_{cm}, y_{cm}, z_{cm})$
   3. **end for**
   4. **for each** $i,j,k$ such that $k=c$, $(i,j) \neq (a,b)$
      1. Get the edge positions of each block in the layer.
      2. $\mathcal{P} \leftarrow$ Compute the support polygons of each block in the layer.
      3. Check if the $(x,y)$ components of $p_d \in \mathcal{P}$.
      4. **if** $p_d \notin \mathcal{P}$ **then**
         1. $\mathcal{B}[d] = \texttt{False}$
         2. **else** $\mathcal{B}[d] = \texttt{True}$
   5. **end for**
4. **end for**
5. **return** $\mathcal{B}$

Note that the algorithm above returns the set of feasbile blocks for **one** turn. The algorithm must be run for each turn, but the tower state must be updated after each turn.

# Gameplay and Decision Making

The computation of the optimal move will be the result of an optimization problem. There are different levels of modeling fidelity that can be incorporated into this optimization problem. The optimization problem is conceptually the following:

$$
\begin{align*}
    \min_{i} \quad -\left \lVert \texttt{Structural stability of the tower} \right \rVert \\
    \text{s.t.} \\
    \texttt{The system remains in static equilibrium.} \\
    \texttt{Block $i$ has not been picked yet.}
\end{align*}
$$

where $i$ is the index of the block. ``Structural stability" is a concept that can be quantified in different ways. For example, it could be measured in terms of:

- The maximum vibration frequency and amplitude that the tower can endure without toppling.
- The maximum vertical force that the can be exerted onto any point from above the tower without it toppling.

When friction and block dimension deviations are incorporated into the modeling framework, the optimization landscape may change accordingly. The cost function will be modified to account for the more realistic physics. These physical effects may also be incorporated into the constraints of the optimization problem. The bottom line is that the optimal move may likely change due to the completeness of the physics modeling framework.

## Algorithm to Update the Tower State

Let us define the tower state as the set of blocks in the tower, and their positions. The tower state is a function of time, because the blocks are removed from the tower. The tower state is also a function of the position of each block, which is a function of time. The tower state $X$ an array, where 

$$
\mathrm{dim}(X) = (\max{\mathcal{K}}) \times \max{\mathcal{J}} \times  2
$$

where the first dimension is the height of the tower, the second dimension is the width of the tower, and the third dimension contains (1) the unique block identifier $d \in \mathcal{N}$ and (2) the center of mass $p_d$ for each block $d$.

The key to updating the tower state is to keep track of the block identifiers. We can do this by using a dictionary, where the keys are the block identifiers, and the values are the positions of the blocks. The algorithm to update the tower state is as follows:

## *Algorithm 2*: Update the Tower State

1. **Input**: Tower state $X$, block mass $m$, width $w$, length $l$, height $h$, and number of blocks $n$.
2. **Output**: Updated tower state $X$.
3. **for** $t = 0$, $t < T$
   1. Move a feasible block according to *Algorithm 1*. Let $d$ be the block identifier.
   2. $\mathcal{K}_t \leftarrow \{1,\dotsc,18 + \mathrm{ceil}(t/3)\}$
4. **end for**

## Strategy for Removing Blocks

### Block Removal Candidates

Conditions for blocks to be removed is as analyses in
previous sections; (1) the block tower must be stable after
target block will be removed, (2) during removing block, two
margins fumargin, flmargin must be positive. Blocks which
satisfy these conditions can be candidate block to remove.
Here, we will select the best block as the largest margin
block. We have the integrated margin:

$$
    f_{margin}^{k,i} = 
    \frac{
        (f_{umargin}^{k,i})(f_{lmargin}^{k,i})
    }
        {f_{umargin}^{k,i} + f_{lmargin}^{k,i}}
$$

A block which has the largest margin will be the candidate
block to be removed. But, as shown in next subsection,
we need second and third candidate. So we set these next
candidate as for the margin.

