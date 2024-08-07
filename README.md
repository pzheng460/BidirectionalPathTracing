# Bidirectional Path Tracing Architecture Summary and Details

## Introduction

This program provides a detailed implementation of a fundamental paper in rendering, focusing on Bidirectional Path Tracing (BDPT), Multiple Importance Sampling (MIS). These techniques are crucial for constructing a "high-end graphics rendering learning system."

## Contents

### 1. Bidirectional Path Tracing Architecture

#### 1.1 Path Weight Calculation

For a path $x = x_0x_1x_2x_3x_4$:

$$ f_j(x) = L_e(x_0 \rightarrow x_1) G(x_0 \leftrightarrow x_1) f_s(x_0 \rightarrow x_1 \rightarrow x_2) G(x_1 \leftrightarrow x_2) f_s(x_1 \rightarrow x_2 \rightarrow x_3) G(x_2 \leftrightarrow x_3) f_s(x_2 \rightarrow x_3 \rightarrow x_4) G(x_3 \leftrightarrow x_4) W_e(x_3 \rightarrow x_4) $$

#### 1.2 Unweighted Path

The unweighted contribution can be written as $C^*_{s,t}$:

$$ C^*_{s,t} \equiv \frac{f_j(x_{s,t})}{p_{s,t}(x_{s,t})} \equiv \alpha^L_s c_{s,t} \alpha^E_t $$

#### 1.3 Path Weights

Using power heuristic weights:

$$ w_k(p_s) = \frac{p_s^2}{\sum_i p_i^2} = \frac{1}{\sum_i \left(\frac{p_i}{p_s}\right)^2} $$

### 2. Importance Flow and Radiance Flow

The light transport equation:

$$ L_o(x, \omega_o) = L_e(x, \omega_o) + \int_{S^2} L_o(M(x, \omega_i), -\omega_i) f_s(x, \omega_i \rightarrow \omega_o) d\sigma^\perp_x(\omega_i) $$

Importance transfer for sensors:

$$ W(x, \omega_o) = W_e(x, \omega_o) + \int_{S^2} W(M(x, \omega_i), -\omega_i) f_s(x, \omega_o \rightarrow \omega_i) d\sigma^\perp_x(\omega_i) $$

### 3. Asymmetric Scattering Issues

Asymmetric scattering occurs with refraction or when shading normals are used. This section discusses handling these issues correctly in BDPT using corresponding adjoint BSDFs.

#### 3.1 Causes of Asymmetric Scattering

When light undergoes refraction, the use of shading normals introduces asymmetry in BSDF calculations.

#### 3.2 Empirical Models and Asymmetry

For example, in the Phong reflection model:

$$ L_o(\omega_o) = \int_{H^2_i} C_r \max(0, \omega_i \cdot M_N(x)(\omega_o))^n L_i(\omega_i) d\sigma(\omega_i) $$

### 4. Refraction Asymmetry

#### 4.1 Refraction BSDF

Deriving explicit formulas for BSDF and adjoint BSDF, and discussing their implications in bidirectional rendering algorithms.

#### 4.2 Adjoint BSDF for Refraction

$$ f^*_s(\omega_i \rightarrow \omega_t) = f_s(\omega_t \rightarrow \omega_i) = \left(\frac{\eta_i}{\eta_t}\right)^2 f_s(\omega_i \rightarrow \omega_t) $$

#### 4.3 Results and Discussion

Proper handling of refraction leads to accurate rendering results. Incorrect scaling in particle tracing results in overly bright caustics.

### 5. Shading Normal Asymmetry

Shading normals can introduce asymmetry by altering BSDF calculations:

$$ f_s(\omega_i \rightarrow \omega_o) = f_{s,N_s}(\omega_i \rightarrow \omega_o) \frac{|\omega_i \cdot N_s|}{|\omega_i \cdot N_g|} $$

## References

1. Veach E. "Robust Monte Carlo Methods for Light Transport Simulation," Ph.D. thesis, Stanford University Department of Computer Science, 1998.
2. Arvo, J. "Analytic Methods for Simulated Light Transport," Ph.D. thesis, Yale University, 1995.
