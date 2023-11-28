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

