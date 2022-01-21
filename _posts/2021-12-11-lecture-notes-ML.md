---
title: "Machine Learning"
excerpt: "Notes on Machine Learning theory. Based on C. M. Bishop, \"Pattern Recognition and Machine Learning\" (2011) and Goodfellow, Bengio, Courville, \"Deep Learning\"."
classes: wide
header:
  teaser: /assets/images/lenet.png
  overlay_image: /assets/images/lenet.png
  overlay_filter: 0.6
  caption: "Photo credit: [**Yann LeCun**](http://yann.lecun.com/)"
  actions:
    - label: "Some Content"
[//]: # (      url: "https://htmlpreview.github.io/?https://github.com/pharath/home/blob/master/_posts_html/2021-09-23-Databases.html")
categories:
  - Lecture_Notes
  - Machine_Learning
tags:
  - lecture_notes
  - ml
toc: true
toc_label: "Contents"

---

# General Remarks

Don't lose sight of the bigger picture ! Only learn stuff when you need it !

> As Feynman said: don't read everything about a topic before starting to work on it. Think about the problem for yourself, figure out what's important, then read the literature. This will allow you to interpret the literature and tell what's good from what's bad. - Y. LeCun

# Overfitting problem

## Why important

TODO

## Paradoxes (related to the overfitting problem)

![sine-taylor](/assets/images/sine-taylor.png)

1. higher order polynomial contains all lower order polynomials as special cases
	- hence, the higher order polynomial **is capable of** generating a result at least as good as the lower order polynomial
		- d.h. das Polynom mit höherer Ordnung **kann** potentiell mindestens genauso gute Ergebnisse liefern ! Wieso passiert das nicht?
2. Sinus-Potenzreihe konvergiert für alle x in R und enthält Terme mit allen Potenzen von x (die geraden Potenzen haben Vorfaktor 0)

Aus 1. und 2. folgt, dass das Ergebnis mit jedem weiteren höheren x-Term **prinzipiell** umso genauer (d.h. näher an der ursprünglichen Sinusfunktion) werden **könnte** (weil die Potenzreihe ja auch mit jedem höheren x-Term genauer wird). Warum passiert das nicht? 

**Antwort**:

1. Taylorreihe und linear model sind nicht vergleichbar: <mark>Die Taylor Reihe approximiert $\sin{x}$ und nicht die Datenpunkte.</mark> D.h. $E > 0$ für die Taylor Reihe, egal wie hoch die Ordnung des Taylor Polynoms ist! Aber das linear model kann mit hinreichend vielen $w_i$ stets einen perfekten Fit, i.e. $E=0$, erzielen.

2. Bei der Potenzreihe (Maclaurin Series) bleiben die weights der ersten N Terme fix, wenn man den Term der Ordnung N+1 hinzufügt, aber beim Fitten nicht ! Die Weights werden beim overfitting zu groß und sorgen für eine schlechtere representation der Targetfunktion als die Potenzreihenrepresentation der Targetfunktion, die die gleiche Ordnung hat!

   Deshalb muss man große weights "bestrafen" (d.h. einen penalty term einfügen aka regularizen), um die weights beim Fitten klein zu halten.

   Wir geben ja den weights völlige Freiheit, aber genau das ist das Problem: die weights fangen beim overfitting an auch den **Noise** in den Daten zu fitten, was für die hohen weight Werte sorgt!

   - **Note**: This happens with decision trees as well: the strongest variables are at the top of the tree, but at the very end, the split that the tree is making, might not be fitting signal but rather noise!

## Possible Solutions (to the overfitting problem)

- reduce number of model parameters
- increase number of data points $N$
- increase regularization parameter lambda (in statistics: shrinkage methods)
    - $q=2$ in 3.29: quadratic regularization: 
        - **advantage**: error function remains a quadratic function of $\mathbf{w}$ $\Rightarrow$ exact minimum can be found in closed form
        - **terminology**:
            - weight decay (in Deep Learning because in sequential learning algorithms, it encourages weight values to decay towards zero, unless supported by the data)
            - L2 regularization = parameter shrinkage = ridge regression (in statistics)
    - $q=1$ in 3.29: "lasso"
        - if $\lambda$ is sufficiently large, some of the coefficients are driven to $0$
            - leads to a **sparse** model in which the corresponding basis functions play no role
                - the origin of this sparsity can be seen [here](/assets/images/bishop_ml/origin_of_sparsity.png)
                    - increasing $\lambda$ corresponds to shrinking the yellow "constraint area". The optimal parameter $\mathbf{w}^*$ corresponds to the point in the yellow area which is closest to the center of the blue circles (center = minimum of unregularized error function) 
                        - [Note: More generally, if the error function does not have circular contours, $\mathbf{w}^*$ would not necessarily be the "closest-to-center" point. In that case we would have to choose the point in the yellow area that minimizes the error function.]
                    - the shape of these yellow "constraint areas" depends on $q$, but their size depends on $\lambda$
                    - the blue circles only depend on the **unregularized** error function, i.e. changing $\lambda$ does not change the blue circles!
- use suitable heuristics (cf. overfitting MoGs)
- include a prior and find a MAP solution (equivalent to adding a regularization term to the error) [see below "Logistic Regression"]

## Related Problems

### In Maximum Likelihood

#### Bias 

- maximum likelihood approach systematically underestimates the **true** variance of the univariate Gaussian distribution (*bias*)
	- this is related to the problem of over-fitting (e.g. too many parameters in polynomial curve fitting):
		> - $\mu_{ML}$ and $\sigma_{ML}^2$ are functions of $x_1, \ldots, x_N$
		> - $x_1, \ldots, x_N$ come from a Gaussian distribution with (separate) $\mu$ and $\sigma^2$
		> - one can show that the expectations of $\mu_{ML}$ and $\sigma_{ML}^2$ wrt this distribution are $\mathbb{E}[\mu_{ML}]=\mu$ and $\mathbb{E}[\sigma_{ML}^2]=\frac{N-1}{N}\sigma^2$, i.e. $\sigma_{ML}$ is biased and underestimates the **true sample variance** $\sigma$
		> - Correcting for this bias yields the **unbiased sample variance** $\tilde{\sigma}^2=\frac{N}{N-1}\sigma_{ML}^2$
- increasing data size $N$ alleviates this problem (and reduces over-fitting [see above])
- this bias lies at the root of the over-fitting problem
- for anything other than small $N$ this bias is not a problem, in practice
- may become a problem for more complex models with many parameters, though

#### MoG<a name="MoG_overfitting"></a>

- singularities in the MoG likelihood will always be present (this is another example of overfitting in maximum likelihood)
    - **Solution**: use suitable heuristics 
        - e.g. detect when a Gaussian component is collapsing and reset mean to a random value and reset covariance to some large value and then continue with the optimization

#### Logistic Regression

- if the data set is linearly separable, ML can overfit
    - magnitude of $\mathbf{w}$ (which corresponds to the "slope" of the sigmoid in the feature space) goes to $\infty$
        - $\Rightarrow$ logistic sigmoid $\to$ Heaviside function
        - $\Rightarrow$ **every** point from each class $k$ is assigned posterior $P(C_k|\mathbf{x})=1$ (i.e. all posteriors are either $0$ or $1$ and there are no points with posteriors **in between** $0$ and $1$)
    - there is a continuum of such solutions
        - ML does not provide a way to favor one specific solution
        - which solution is found will depend on:
            - parameter initialization
            - choice of optimization algorithm
- **Solution**:
    - use a prior and find a MAP solution for $\mathbf{w}$
    - or equivalently: add a regularization term to the error function

# Probability Theory

## Why important?

- <mark>**[Bayesian Inference](https://en.wikipedia.org/wiki/Bayesian_inference):**</mark> We have to understand the concepts of **posterior** and **prior** probabilities, as well as **likelihood** because e.g. both generative and discriminative linear models model the posterior probabilities $P(C_k\vert\mathbf{x})$ of the classes $C_k$ given the input $\mathbf{x}$. 

    > "Bayesian inference is a method of statistical inference in which Bayes' theorem is used to update the probability for a hypothesis as more evidence or information becomes available." - [Wikipedia](https://en.wikipedia.org/wiki/Bayesian_inference)

- <mark>**Probability distributions:**</mark> 
    - Terminology:
        - probability distribution: overall **shape** of the probability density 
        - probability density function, or PDF: probabilities for specific outcomes of a random variable
    - these are useful for:
        - **Outlier detection**: It is useful to know the PDF for a sample of data in order to know whether a given observation is unlikely, or so unlikely as to be considered an **outlier or anomaly** and whether it should be removed. 
        - **in order to choose appropriate learning methods** that require input data to have a specific probability distribution.
        - for  **probability density estimation** (see below).
        - distributions can form building blocks **for more complex models** (e.g. MoG)

- <mark>**Density estimation:**</mark> Sometimes we want to model the probability distribution $P(\mathbf{x})$ of a random variable $\mathbf{x}$, given a finite set $\mathbf{x}_1, \ldots, \mathbf{x}_N$ of observations (= **density estimation**). 
    - there are infinitely many probability distributions that could have given rise to the observed data, i.e. the density estimation problem is **fundamentally ill-posed**
    - <mark>important because</mark> e.g. naive Bayes classifiers coupled with kernel density estimation (see below) can achieve higher accuracy levels.
    - 2 density estimation methods:
        - **parametric density estimation**:<a name="parametric_density"></a>
            1. choose **parametric distribution**:
                - discrete random variables:
                    - Binomial distribution
                    - Multinomial distribution
                - continuous random variables:
                    - Gaussian distribution
            2. **<mark>Learning</mark>**: estimate parameters of the chosen distribution given the observed data set
                - frequentist:
                    - via Maximum Likelihood
                - Bayesian:
                    1. introduce prior distributions over the parameters
                    2. use Bayes' theorem to compute the corresponding posterior over the parameters given the observed data
                    - Note: This approach is simplified by [**conjugate priors**](https://en.wikipedia.org/wiki/Conjugate_prior). These lead to posteriors which have the same functional form as the prior. Examples:
                        - conjugate prior for the parameters of the multinomial distribution: **Dirichlet distribution**
                        - conjugate prior for the mean of the Gaussian: Gaussian
        - **nonparametric density estimation**:
            - size of the data set $N$ determines the **form of the distribution**, parameters determine the **model complexity** (i.e. "flexibility", similar to the order of the polynomial in the polynomial curve fitting example) (but not the form of the distribution!)
            - histograms
            - nearest-neighbours (e.g. K-NN)
            - kernels (e.g. KDE)

Hence, understanding the basic concepts of probability theory is crucial for understanding linear models.

## Sum rule, Product rule, Bayes' Theorem

### Example: Fruits in Boxes

- Let $P(B=r) = 0.4$ and $P(B=b) = 0.6$ (**frequentist view**: this is how many times the guy who picked the fruit picked each box [in the limit of infinitely many pickings], i.e. the picker had a tendency to pick the blue one more often). 
	- Suppose that we pick a box at random, and it turns out to be the blue box. What's the probability of picking an apple?
	- What's the (overall) probability of choosing an apple?
	- Suppose we picked an orange. Which box did it come from?
- **prior** vs **posterior** explain intuitively (e.g. "evidence favouring the red box")
- **independence** intuitively (e.g. "independent of which box is chosen")

## Likelihood

- $P(\mathbf{x}\vert\vec{\theta})=L_{\mathbf{x}}(\vec{\theta})$ expresses how probable the observed data point $\mathbf{x}$ is (as a function of the parameter vector $\vec{\theta}$)
    - the likelihood is not a probability distribution **<mark>over w</mark>**, and its integral **<mark>with respect to w</mark>** does not (necessarily) equal one.
        - not $\mathbf{x}$ because integral w.r.t. $\mathbf{x}$ **is** $1$.
- **MoGs**: Similarly, in MoGs $P(\mathbf{x}\vert k)$ expresses the likelihood of $\mathbf{x}$ given mixture component $k$
    - i.e. the MoG likelihood itself is composed of several individual Gaussian likelihoods

## Frequentist vs Bayesian approach

- there is no unique frequentist or Bayesian viewpoint

### Frequentist

- Maximum Likelihood views true parameter vector $\theta$ to be unknown, but <mark>fixed</mark> (i.e. $\theta$ is **not** a random variable!).

### Bayesian

- In Bayesian learning <mark>$\theta$ is a random variable</mark>.
- prior encodes knowledge we have about the type of distribution we expect to see for $\theta$ 
    - (e.g. from calculations how fast the polar ice is melting)
- history: 
    - Bayesian framework has its origins in the 18th century, but the practical application was <mark style="color: white; background-color: red; opacity: 1">limited by</mark> "the difficulties in carrying through the full Bayesian procedure, particularly <mark style="color: white; background-color: red; opacity: 1">the need to marginalize (sum or integrate) over the whole of parameter space</mark>, which [...] is required in order to make predictions or to compare different models" (Bishop)
        - [technical developments](/assets/images/bishop_ml/history_development_bayes_approach.png) changed this!
- critics:
    - prior distribution is often selected on the basis of mathematical convenience rather than as a reflection of any prior beliefs $\Rightarrow$ subjective conclusions
        - one motivation for so-called **noninformative priors**
        - Bayesian methods based on poor choices of prior can give poor results with high
confidence
- advantages (compared to frequentist methods):
    - inclusion of prior knowledge
        - cf. example "tossing a coin three times and landing heads three times": 
            - Bayesian approach gives a better estimate of the probability of landing heads (whereas Maximum Likelihood would give probability 1)

## The Uniform Distribution

- Mean: $\frac{(a+b)}{2}$ [[derivation]](https://de.wikipedia.org/wiki/Stetige_Gleichverteilung#Erwartungswert_und_Median)
- Variance: $\frac{1}{12}(b-a)^2$ [[derivation]](https://de.wikipedia.org/wiki/Stetige_Gleichverteilung#Varianz)

## The Gaussian Distribution

> "Of all probability distributions over the reals with a specified mean $\mu$ and variance $\sigma^2$, the normal distribution $N(\mu,\sigma^2)$ is the one with **maximum entropy**." - [Wikipedia](https://en.wikipedia.org/wiki/Normal_distribution#Maximum_entropy)

- this also holds for the **multivariate** Gaussian (i.e. the multivariate distribution with maximum entropy, for a given mean and covariance, is a Gaussian)
    - Proof: 
        - maximize the entropy of a distribution $p(x)$ $H[x]$ over all distributions $p(x)$ (likelihoods) subject to the constraints that $p(x)$ be normalized and that it has a specific mean and covariance
        - the maximum likelihood distribution is given by the multivariate Gaussian

### Central Limit Theorem (CLT)

Subject to certain mild conditions, the sum of a set of random variables (which is itself a random variable) has a distribution that becomes increasingly Gaussian as the number of terms in the sum increases.

- Let $X_1+\ldots+X_n$ be i.i.d. random variables. The expectation of $S_n=X_1+\ldots+X_n$ is $n\mu$ and the variance is $n\sigma^2$. [[source]](https://de.wikipedia.org/wiki/Zentraler_Grenzwertsatz#Der_Zentrale_Grenzwertsatz_der_Statistik_bei_identischer_Verteilung)

#### Example: Sum of uniformly distributed i.i.d. random variables (e.g. distribution of the average of random variables)

$x_1,\ldots,x_N\sim \mathcal{U}[0,1]\xrightarrow[]{N \to \infty} \frac{1}{N}(x_1+\ldots +x_N)\sim\mathcal{N}(\frac{1}{2},\frac{1}{12N})$

- [[Proof]](/assets/images/proofs/variance_of_sum_of_Uniformly_distributed_iid_RV.png) for the mean $\frac{1}{2}$ and the variance $\frac{1}{12N}$, where $\bar{X}=\frac{1}{N}(x_1+\ldots +x_N)$ [[source]](https://www.math.umd.edu/~millson/teaching/STAT400fall18/slides/article21.pdf)
    - Recall the rules for expectations and variances.
- Notation: The $[0,1]$ in $\mathcal{U}[0,1]$ denotes an interval, whereas the $(\frac{1}{2},\frac{1}{12N})$ in $\mathcal{N}(\frac{1}{2},\frac{1}{12N})$ denotes the mean and the variance of the Gaussian (specifying an interval for the Gaussian does not make sense)

### Relation to other distributions

- Binomial distribution $\xrightarrow[]{N\to\infty}$ Gaussian distribution

![Binomial_Gaussian_Poisson](/assets/images/binomial_gaussian_poisson.jpg)

### Intuition for Multivariate Gaussians

- spectral decomposition of the covariance $\Sigma$
- Shape of the Gaussian:
    - scaling given by $\lambda_i$, 
    - shifted by $\mathbf{\mu}$, 
    - rotation given by the eigenvectors $\mathbf{u}_i$
- new coordinate system defined by eigenvectors $\mathbf{u}_i$ with axes $y_i=\mathbf{u}_i^T(\mathbf{x}-\mathbf{\mu})$ for $i=1,\ldots,D$
    > "In linear algebra, **eigendecomposition** is the factorization of a matrix into a canonical form, whereby the matrix is represented in terms of its eigenvalues and eigenvectors. Only diagonalizable matrices can be factorized in this way. When the matrix being factorized is a normal or real symmetric matrix, the decomposition is called **"spectral decomposition"**, derived from the spectral theorem." - [Wikipedia](https://en.wikipedia.org/wiki/Eigendecomposition_of_a_matrix) 
    - origin of new coordinate system is at $\mathbf{y}=\mathbf{0}$, i.e. at $\mathbf{x}=\mathbf{\mu}$ in the old coordinate system!
- **Normalization:** one can show that ![eq_2_56](/assets/images/equations/eq_2_56.png)
    i.e. in the EV coordinate system the joint PDF factorizes into $D$ independent **univariate** Gaussians! The integral is then $\int{p(\mathbf{y})d\mathbf{y}}=1$, which shows that the multivariate Gaussian is normalized.
- **1st order and 2nd order moments**:
    - Note: 2nd order moment (i.e. $\mu\mu^T+\Sigma$) $\neq$ covariance of $\mathbf{x}$ (i.e. $\Sigma$)
- **Number of independent parameters**: $\Sigma$ has in general $D(D+1)/2$ independent parameters
    > - $D^2-D$ non-diagonal elements of which $(D^2-D)/2$ are unique
    > - $D$ diagonal elements
    > - i.e. in total $(D^2-D)/2+D=D(D+1)/2$ independent parameters in $\Sigma$
    - <mark style="color: white; background-color: red; opacity: 1">grows quadratically with $D$</mark> $\Rightarrow$ manipulating/inverting $\Sigma$ can become computationally infeasible for large $D$
    - $\Sigma$ and $\mu$ together have $D(D+3)/2$ independent parameters
- **special cases of covariance matrices**:
    - general $\qquad\qquad\qquad\qquad\Rightarrow D(D+3)/2$ independent parameters (shape: hyperellipsoids)
    - diagonal $\Sigma=\mathop{\mathrm{diag}}(\sigma_i)$ $\qquad\Rightarrow 2D$ independent parameters (shape: axis-aligned hyperellipsoids)
    - isotropic $\Sigma=\sigma^2\mathbf{I}$ $\qquad\qquad\Rightarrow D+1$ independent parameters (shape: hyperspheres)

### Limitations of Gaussians

#### Problem

- as discussed above, the Gaussian can be both
    - too flexible (too many parameters) and 
    - too limited in the distributions that it can represent 
        - Gaussian is unimodal, i.e. it has a single maximum, but we want to approximate multimodal distributions!

#### Solution: Latent variable models 

- use latent (hidden, unobserved) variables 
    - discrete latent variables $\Rightarrow$ MoGs (which are multimodal!)
    - continuous latent variables $\Rightarrow$ ["probabilistic PCA"](/assets/images/bishop_ml/probabilistic_PCA.png), [factor analysis](/assets/images/bishop_ml/factor_analysis.png) 
        > "<mark>Probabilistic PCA represents</mark> a constrained form of <mark>the Gaussian distribution</mark> in which the number of free parameters can be restricted while still allowing the model to capture the dominant correlations in a data set." - Bishop, "Pattern Recognition and Machine Learning"
        - i.e., like MoGs, probabilistic PCA is a parametric density estimation method (see [above](#parametric_density)) 
        - see also [here](/assets/images/bishop_ml/simplest_contin_latent_var_model_1.png) and [here](/assets/images/bishop_ml/simplest_contin_latent_var_model_2.png)
        - probabilistic PCA and factor analysis are related
        - formulation of PCA as a probabilistic model was proposed independently by
Tipping and Bishop (1997, 1999b) and by [Roweis (1998)](http://www.stat.columbia.edu/~liam/teaching/neurostat-fall20/papers/hmm/roweis-ghahramani-lds.pdf) $\rightarrow$ see "Linear Gaussian Models"
            - also important [applications in control theory](https://adam2392.github.io/blog/2019/06/gaussian-generative-models/)
        - [[paper: GMMs vs Mixtures of Latent Variable Models]](https://publications.idiap.ch/attachments/reports/2000/rr00-25.pdf)
            > "One of the most popular density estimation methods is the Gaussian mixture model (GMM). <mark>A promising alternative to GMMs [= MoGs] are the recently proposed mixtures of latent variable models. Examples of the latter are principal component analysis and factor analysis.</mark> The advantage of these models is that they are capable of representing the covariance structure with less parameters by choosing the dimension of a subspace in a suitable way. An empirical evaluation on a large number of data sets shows that mixtures of latent variable models almost always outperform various GMMs both in density estimation and Bayes classifiers." - from [[paper: GMMs vs Mixtures of Latent Variable Models]](https://publications.idiap.ch/attachments/reports/2000/rr00-25.pdf)
- Latent variable models:
    - discrete latent variables
        - continuous $\mathbf{x}$:
            - MoG/GMM
        - discrete $\mathbf{x}$:
            - Mixture of Bernoulli distributions
    - continuous latent variables
        - probabilistic PCA
        - factor analysis

### Proof: Covariance $\Sigma$ symmetric w.l.o.g.

![proof-Gaussian-cov-symmetric-wlog](/assets/images/proofs/Gaussian_cov_symmetric_wlog.png)

- **<mark>Key point:</mark>** If $\mathbf{A}=\mathbf{\Sigma}^{-1}$ is not symmetric, then there is another symmetric matrix $\mathbf{B}$ so that $\Delta^2=(\mathbf{x}-\mathbf{\mu})^T\mathbf{\Sigma}^{-1}(\mathbf{x}-\mathbf{\mu})$ is equal to $\Delta^2=(\mathbf{x}-\mathbf{\mu})^T\mathbf{B}(\mathbf{x}-\mathbf{\mu})$.
- **Why is this important?**
    - If $\Sigma$ was not symmetric, its eigenvectors would not necessarily form an orthonormal basis. Hence, the above intuition for the Gaussians would not hold.
    - If $\Sigma$ is a real, symmetric matrix its eigenvalues are real and its eigenvectors can be chosen to form an orthonormal basis ([symmetric matrices/diagonalizable](https://de.wikipedia.org/wiki/Symmetrische_Matrix#Diagonalisierbarkeit) and [symmetric matrices/orthoganally diagonalizable](https://de.wikipedia.org/wiki/Symmetrische_Matrix#Orthogonale_Diagonalisierbarkeit)) 
        - in other words, it's important in order to apply the spectral theorem:
            > "The finite-dimensional **spectral theorem says** that any symmetric matrix whose entries are real can be diagonalized by an orthogonal matrix. More explicitly: For every real symmetric matrix $A$ there exists a real orthogonal matrix $Q$ such that $D = Q^{\mathrm{T}} A Q$ is a diagonal matrix. Every real symmetric matrix is thus, up to choice of an orthonormal basis, a diagonal matrix." - Wikipedia

## MoG

- MoGs are probabilistic generative models (see below), if we view each mixture component $k$ as a class
- parameters are determined by:
    - Frequentist:
        - Maximum Likelihood:
            - **problem 1**: no closed-form analytical solution
                - **solutions**: 
                    - iterative numerical optimization
                        - gradient-based optimization
                    - EM algorithm
            - **problem 2**: maximization of MoG likelihood is not a well posed problem because "**singularities**" will always be present
                - **solution**:
                    - see section [Overfitting in Maximum Likelihood](#MoG_overfitting)
            - **problem 3**: "**identifiability**": $K!$ equivalent solutions
                - important issue when we want to interpret the parameter values
                - however, not relevant for density estimation
    - Bayesian:
        - Variational Inference
            - little additional computation compared with EM algorithm
            - resolves the principle difficulties of maximum likelihood
            - allows the number of components to be inferred **automatically** from the data
- can be written in two ways:
    - standard formulation
    - latent variable formulation [[9.10-12]](/assets/images/equations/eq_9_10_to_12.png)
        - **why useful?**
            - for ancestral sampling
            - **simplifications of the ML solution of MoG**<a name="MoG_ML_lat_var_form"></a>: we can work with the joint distribution $P(\mathbf{x},\mathbf{z})$ instead of the marginal $P(\mathbf{x})$ $\Rightarrow$ we can write the **complete-data** log-likelihood in the form
                - simplifies maximum likelihood solution of MoG
                    - using latent variables the complete-data log-likelihood can be written as [[9.35]](/assets/images/equations/eq_9_35.png), which is basically $p(\mathbf{X},\mathbf{Z})=\prod_{n=1}^N p(\mathbf{z}_n)p(\mathbf{x}_n\vert\mathbf{z}_n)$ (recall: there is one $\mathbf{z}_n$ for each $\mathbf{x}_n$)
                        - do not sum over $\mathbf{z}$ as in [[9.12]](/assets/images/equations/eq_9_10_to_12.png)! 9.12 is $p(\mathbf{x})$ and not $p(\mathbf{x},\mathbf{z})$! Simply insert [[9.10]](/assets/images/equations/eq_9_10_to_12.png) and [[9.11]](/assets/images/equations/eq_9_10_to_12.png) in $p(\mathbf{X},\mathbf{Z})$.
                        - this leads to a **closed form** solution for the MoG maximum likelihood (which is useless in practice, though)
                            - maximization w.r.t. mean and covariance is exactly as for the single Gaussian, except that it involves only the data points that are assigned to each component. The mixing coefficients are equal to the fractions of data points assigned to the corresponding components.
                                - this simplification would not be possible **without** the introduction of latent variables!
                            - however, the latent variables are not known, in practice
                                - but we can maximize the **expectation of the complete-data log likelihood function** (w.r.t. the posterior distribution of the latent variables) [[9.40]](/assets/images/equations/eq_9_40.png)
                                    - we can calculate this expectation by choosing some initial parameters in order to calculate $\gamma(z_{nk})$ and then maximizing [[9.40]](/assets/images/equations/eq_9_40.png) w.r.t. the parameters (while keeping the responsibilities fixed)
                                    - $\Rightarrow$ EM algorithm
                        - Note: the MoG likelihood **without** latent variables (see 9.14) **cannot** be maximized in closed form, but the EM algorithm gives closed form expressions for $\mu_k$, $\Sigma_k$ and $\pi_k$. 
                            - ("closed form expression" only means "a mathematical expression that uses a finite number of standard operations" and hence, $\mu_k$, $\Sigma_k$ and $\pi_k$ are in closed form) 
                        - Interpreting 9.36 is much easier than interpreting 9.14!

### Mixture Models in general

- use cases:
    - framework for building more complex probability distributions
    - clustering data
        - clustering with hard assignments (K-means algorithm)
        - clustering with soft assignments (EM algorithm)

#### Applications of Mixture Models

- applications typically model the distribution of pixel colors and learn a MoG to represent the class-conditional densities:
    - image segmentation
        - mark two regions to learn MoGs and classify background VS foreground
    - tracking
        - train background MoG model with an **empty** scene (i.e. the object to be tracked is not in this scene at first) in order to model common appearance variations for each pixel
        - anything that cannot be explained by this model will be labeled as foreground, i.e. the object to be tracked
        - Note: in order to adapt to e.g. lighting changes the MoG can be updated over time

# Outlier removal

- by choosing a suitable nonlinearity (e.g. sigmoid, tanh)
- remove outlier data point completely from the data set

# Model Selection / Comparison

- TODO

# The Curse of Dimensionality

## Why important?

- For all classifiers, the more dimensions we have in the input space (= **feature space**), the more training data we need.
    - Consider the simple classifier described in Bishop, Chapter 1.4 "The Curse of Dimensionality". 
        - The problem with an exponentially large number of cells in the input space is that we would need an exponentially large number of training data points in order to ensure that the cells are not empty. Therefore, applying this classifier to more input variables (here: only two input variables (= **features**): $x_6$ and $x_7$) becomes infeasible.

> "In machine learning problems [...] typically an enormous amount of training data is required to ensure that there are several samples with each combination of values. A typical rule of thumb is that there should be at least 5 training examples for each dimension in the representation. [...] This ["Curse of Dimensionality"] phenomenon states that with a fixed number of training samples, <mark>the average (expected) predictive power of a classifier</mark> or regressor <mark>first increases</mark> as the number of dimensions or features used is increased but <mark>beyond a certain dimensionality it starts deteriorating</mark> instead of improving steadily." - [Wikipedia](https://en.wikipedia.org/wiki/Curse_of_dimensionality#Machine_Learning)

- TODO

# Decision Theory

## Why important?

The **classification problem** can be broken down into two stages:
- **Inference Stage**: use training data to learn a model for $P(C_k\vert\mathbf{x})$
- **Decision Stage**: use these $P(C_k\vert\mathbf{x})$ to make optimal class assignments

Generative and discriminative models use this "two stage" approach. Discriminant functions do **not** have access to the posteriors $P(C_k\vert\mathbf{x})$! 

# ML for Mixture Models

## K-means algorithm

- corresponds to a particular nonprobabilistic limit of EM applied to MoGs (see EM algorithm)
- **goal**: find values for all $r_{nk}$ and $\pmb{\mu}_k$ so as to minimize $J$ (= "sum of all distances")
---
**K-means intuitively**

- E step: determine all $r_{nk}$ ("assign $\mathbf{x}_n$ to closest cluster center")
- M step: determine all $\pmb{\mu}_k$ ("set $\pmb{\mu}_k$ equal to the mean of all $\mathbf{x}_n$ assigned to cluster $k$")

---
- Note: this algorithm is guaranteed to reduce the value of the objective function $J$ in each phase and hence, is **guaranteed to converge after a finite number of iterations**
    - $J$ corresponds to the negative expectation of the complete-data log likelihood in the EM algorithm, see 9.43 and disussion of general EM algorithm

### Initialization of K-means

- choose the initial $\pmb{\mu}_k$ to be equal to a random subset of $K$ data points

### Advantages 

- simple and fast to compute
- guaranteed convergence in finite number of iterations
    - convergence faster than for standard EM algorithm

### Issues in practice

- may converge to a local rather than global minimum (like the EM algorithm)
    - i.e. the final result **depends on initialization**
- E step may be slow because it requires computing all distances
    - **solution**: speed up by
        - precompute a data structure (e.g. a tree, where nearby points are in same subtree)
        - avoid unnecessary distance calculations (e.g. using the triangle inequality)
- detects spherical clusters only
- sensitive to outliers
- choosing $K$

### Online version

- above a batch version of K-means is described, but there is also a sequential update online version
- via Robbins-Monro procedure ([MacQueen, 1967](https://projecteuclid.org/ebooks/berkeley-symposium-on-mathematical-statistics-and-probability/Some-methods-for-classification-and-analysis-of-multivariate-observations/chapter/Some-methods-for-classification-and-analysis-of-multivariate-observations/bsmsp/1200512992))
    - see Bishop, 2.3.5 "Sequential estimation"

### K-medoids algorithm

- uses other dissimilarity measure $J$
    - hence, its M step is potentially more complex
        - **solution**: restrict each cluster prototype to be equal to one of the $\mathbf{x}_n$ assigned to that cluster
- **advantages compared to K-means**:
    - cluster means are more robust to outliers
    - limits the data types that can be considered
        - e.g. for categorical inputs a Euclidean distance cannot be calculated

### Complexity

- E step: $\mathcal{O}(KN)$ (for both K-means and K-medoids)
- M step: $\mathcal{O}(N_k^2)$ for each cluster $k$

### Elliptical K-means

- **idea**: EM gives an estimate for $\Sigma$, however, standard K-means (which is a special case of an EM algorithm) does not estimate $\Sigma$
    - $\Rightarrow$ hard assignments with general $\Sigma$ instead of $\Sigma=\epsilon\mathbf{I}$ for $\epsilon\to 0$ (Sung, Poggio, 1994)

### Applications of K-means

- Image Segmentation
- Image Compression 
    - using **vector quantization**

## Latent variable formulation of Mixture Distributions

- this is basically an alternative (i.e. equivalent) formulation of mixture models (e.g. MoGs) that simplifies certain calculations (see e.g. "latent variable view of EM algorithm")
- Merke: <mark>"For every observed data point $\mathbf{x}_n$ there is a corresponding latent variable $\mathbf{z}_n$."</mark>
    - i.e. there are $N$ latent variables $\mathbf{z}_n$, so to speak
    - $\mathbf{z}_n$ is not known
        - in ancestral sampling $\mathbf{z}$ is known because we sample from $P(\mathbf{z})$ first, so we know the value of $\mathbf{z}$
            - and, therefore, we know the component that generates $\mathbf{x}$ 
                - corresponding to the $z_k$ which is equal to $1$
    - $\mathbf{z}_n$ encodes which mixture component $\mathbf{x}_n$ belongs to
- $\{\mathbf{X},\mathbf{Z}\}$ is called **complete** data set 
    - the corresponding log-likelihood is $\ln P(\mathbf{X},\mathbf{Z}\vert\mathbf{\vec{\theta}})$, where $P(\mathbf{X},\mathbf{Z}\vert\mathbf{\vec{\theta}})=\prod_{n=1}^N P(\mathbf{z}_n)P(\mathbf{x}_n\vert\mathbf{z}_n)$
        - this is basically a generalization of $\ln P(\mathbf{X}\vert\mathbf{\vec{\theta}})$ <mark>which simplifies the Maximum Likelihood treatment for MoGs</mark> (in theory, not in practice, see [theoretical ML for MoG](#MoG_ML_lat_var_form)) 
- actual observed data $\mathbf{X}$ is called **incomplete**
- can be interpreted as defining assignments of data points to specific components of the mixture

## EM algorithm

- **goal**: find maximum likelihood solutions for <mark>models having latent variables</mark>
- use cases:
    - find MAP solutions for models in which a prior $P(\mathbf{\vec{\theta}})$ is defined
    - **Handling missing values**: can also be applied when the unobserved variables correspond to missing values in the data set
        - only applicable, if values are missing at random (MAR)
            > "Under the classical missing at random mechanism (MAR) assumption, the parameters can thus be estimated by maximizing the observed likelihood. To do so, it is possible to use an Expectation-Maximization (EM) algorithm (Dempster, Laird, and Rubin, 1977) [...]." - [source: section 11.1.1.3 Two recommended methods: EM / Multiple imputation](https://julierennes.github.io/MAP573/handling-missing-values.html)
---
**General EM Algorithm**

- **E step**: 
    - find $P(\mathbf{Z}\vert\mathbf{X},\pmb{\theta}^{old})$ 
        - in the standard EM algorithm this corresponding to: find $\gamma(z_{nk})$
    - find the expectation (w.r.t this $P(\mathbf{Z}\vert\mathbf{X},\pmb{\theta}^{old})$) of the complete-data log likelihood $Q(\pmb{\theta},\pmb{\theta}^{old})$ 
- **M step**: update $\pmb{\theta}^{new}=\argmax_{\pmb{\theta}}Q(\pmb{\theta},\pmb{\theta}^{old})$ 

---
- Note: this algorithm is guaranteed to increase the incomplete-data log likelihood in each cycle

### Initialization of EM

1. Run K-means (e.g. M times) 
2. pick the best result (i.e. lowest error $J$)
3. initialize EM parameters:
    - initialize $\Sigma_k$ to $\Sigma$ of clusters found by K-means
    - initialize $\pi_k$ to fraction of $\mathbf{x}$ assigned to clusters found by K-means

### Issues in practice

- **MoG singularities**: employ techniques to avoid singularities (see [overfitting MoGs](#MoG_overfitting))
- **multiple local maxima**: the log likelihood has in general multiple local maxima 
    - the EM algorithm is not guaranteed to converge to the largest maximum!

### Credit Assignment problem

- **problem**: when we are given $\mathbf{x}_n$, we don't know which component generated this point
- **solution**: we can calculate the responsibilities of each component $\gamma_k(\mathbf{x}_n)=\gamma(z_{kn})=p(z_k=1\vert\mathbf{x}_n)$ for explaining $\mathbf{x}_n$
    - corresponds to a **soft assignment** of each sample $\mathbf{x}_n$ to a mixture component $\Rightarrow$ this is why the EM algorithm is sometimes referred to as "**Clustering with soft assignments**"
        - $\mathbf{x}_n$ is assigned to **all** mixture components (with some probability $\gamma_k(\mathbf{x}_n)$ for each component) instead of only one component (cf. hard assignments of the K-means algorithm)
    - <mark>This is what the E-step of the EM algorithm does!</mark>

### Relation to K-means

- corresponds to a particular nonprobabilistic limit of EM applied to MoGs
    - consider a MoG where each mixture component has covariance $\epsilon\mathbf{I}$ and thus looks like [[9.41]](/assets/images/equations/eq_9_41.png)
        - then the responsibilities $\gamma(z_{nk})$ for a data point $\mathbf{x}$ all go to zero except for one which will go to unity (in other words: $\gamma(z_{nk})\to r_{nk}$)
            - this corresponds to a hard assignment of the data point $\mathbf{x}_n$
        - the $\pmb{\mu}_k$ will go to the K-means $\pmb{\mu}_k$
        - the $\pi_k$ will be reset to the fraction of data points assigned to cluster $k$ (as usual in the EM algorithm) 
            - however, the $\pi_k$ are irrelevant for the algorithm now
        - the expected complete-data log likelihood $Q(\pmb{\theta},\pmb{\theta}^{old})$ will go to the negative of the distortion measure $J$ for the K-means algorithm

# Linear models for regression

- the polynomial is one example of a broad class of functions called **linear regression models**
- linear regression models:
    - <mark>are always **linear functions of the parameters**</mark>

# Linear models for classification

**Difference to DNNs/MLPs**: linear models use **fixed** basis functions, whereas DNNs/MLPs use **learned** basis functions [via hidden layers]

> Note:
> - linear models (i.e. <mark>WITHOUT</mark> activation function) 
>     - these models are linear in the parameters! 
>     - the decision surfaces are **linear** functions of the input vector $\mathbf{x}$ (or of the feature vector $\pmb{\phi}(\mathbf{x})$, see [LBFs](#LBF))
>         - ($D-1$-dimenstional hyperplanes) 
> - generalized linear models (i.e. <mark>WITH</mark> activation function)
>     - these models are **not** linear in the parameters! 
>         - (in cotrast to the linear models for regression discussed in [Bishop_2006](#Bishop_2006))
>     - the decision surfaces are **linear** functions of the input vector $\mathbf{x}$ (or of the feature vector $\pmb{\phi}(\mathbf{x})$, see [LBFs](#LBF))
>         - ($D-1$-dimenstional hyperplanes) 
>         - because decision surfaces correspond to $y(\mathbf{x})=\text{const}$, which implies $\mathbf{w}^\top\mathbf{x}+w_0=\text{const}$
>     - Open Questions: 
>         - lecture slides: if the activation function is not monotonous, the decision surface is not a linear function of $\mathbf{x}$, why?
>     - why decision surface**s** plural and not singular?

## discriminant functions

> Note: [Bishop_2006](#Bishop_2006) only discusses **linear** discriminants in chapter 4.1 (i.e. the decision surfaces are $D-1$-dimensional hyper**planes**) which does not mean that discriminant functions must always be linear! In particular, 
> - the introduction to chapter 4 [mentions](/assets/images/bishop_ml/discriminant_functions_intro.png) that all algorithms discussed in this chapter are equally applicable using nonlinear transformations $\pmb{\phi}$.
> - this is also mentioned in chapter "4.3.1 Fixed basis functions" 

- 2 classes
    - single 2-class discriminant $y(\mathbf{x})=\mathbf{w}^\top\mathbf{x}+w_{0}$
    - equivalently: single 2-class discriminant comprising $2$ linear functions $y_k(\mathbf{x})=\mathbf{w}^\top_k\mathbf{x}+w_{k0}$
    - Rosenblatt's perceptron
- K classes
    - one-vs-rest
    - one-vs-one
    - single K-class discriminant comprising $K$ linear functions $y_k(\mathbf{x})=\mathbf{w}^\top_k\mathbf{x}+w_{k0}$
        - decision regions are convex (proof in [Bishop_2006](#Bishop_2006)) 
            - "Every convex subset of $\mathbb{R}^n$ is simply [= singly] connected." - [Wikipedia](https://en.wikipedia.org/wiki/Simply_connected_space)
                - $\Rightarrow$ decision regions are also singly connected 
- learning the parameters
    - least squares
        - wrong tool for binary (i.e. 1-of-K coded) targets because binary targets do not have a Gaussian distribution and least squares corresponds to ML under the assumption of a Gaussian target distribution (see [Bishop_2006](#Bishop_2006), 1.2.5)
    - Fisher's linear discriminant
    - perceptron algorithm

## probabilistic generative models (indirect modeling of posterior)

- $P(C_k\vert\mathbf{x})$ can be written as logistic sigmoid [4.57](/assets/images/equations/eq_4_57.png)
    - i.e. $P(C_k\vert\mathbf{x})$ has a sigmoidal shape (when viewed as function of $\mathbf{x}$), **if** "$a$" [4.58](/assets/images/equations/eq_4_58.png)    
    is linear in $\mathbf{x}$!
- first model $P(\mathbf{x}\vert C_k)$ and $P(C_k)$, then use [4.57](/assets/images/equations/eq_4_57.png)-[4.58](/assets/images/equations/eq_4_58.png) to find $P(C_k\vert\mathbf{x})$ (or use equivalently Bayes' theorem [1.82](/assets/images/equations/eq_1_82.png) and [1.83](/assets/images/equations/eq_1_83.png))
    - Examples:
        - **Continuous Inputs**: (Gaussian distribution)
            - model each $P(\mathbf{x}\vert C_k)$ as a Gaussian [4.64](/assets/images/equations/eq_4_64.png), <mark>where all classes share the same $\pmb{\Sigma}$</mark> $\Rightarrow$ posterior $P(C_k\vert\mathbf{x})$ is the logistic sigmoid [4.65](/assets/images/equations/eq_4_65.png) (2 classses) or the softmax [4.62]() ($K\geq2$ classes) where $a_k$ is given by [4.68](), i.e. a **generalized linear model**
                - (i.e. linear decision boundaries, but not linear in parameters!)
                - decision boundaries are where (the 2 largest) posteriors are equal
                - use Maximum Likelihood to determine parameters of Gaussian 4.64 and priors $P(C_k)$ (requires data set)
                - priors enter only through $w_0$ [4.67](/assets/images/equations/eq_4_67.png)
                    - i.e. priors shift the decision boundary parallelly (vgl. [4.65](/assets/images/equations/eq_4_65.png) mit distance from the origin to the decision surface [4.5](/assets/images/equations/eq_4_5.png))
                    - i.e. priors shift the parallel contours of constant posterior probability
                - the argument of the sigmoid (2 classes) or the $a_k(\mathbf{x})$ of the softmax ($K\geq2$ classes) are linear functions of the inputs $\mathbf{x}$
                - Note: if the classes do **not** share the same $\pmb{\Sigma}$, the decision boundaries will be **quadratic**, i.e. the $P(C_k|\mathbf{x})$ are **not** governed by a generalized linear model!
        - **Discrete Inputs**: (Bernoulli distribution)
            - model $P(\mathbf{x}\vert C_k)$ as [Bernoulli naive Bayes](#https://en.wikipedia.org/wiki/Naive_Bayes_classifier#Bernoulli_na%C3%AFve_Bayes) model $P(\mathbf{x}\vert C_k)=\prod_{i=1}^Dp_{ki}^{x_i}(1-p_{ki})^{(1-x_i)}$ $\Rightarrow$ posterior $P(C_k\vert\mathbf{x})$ a logistic sigmoid (2 classses) or the softmax [4.62]() ($K\geq2$ classes) where $a_k$ is given by [4.82](), i.e. again a **generalized linear model**
                - $x_i\in\{0,1\}$ (e.g. "spam or ham")
                - $p_{ki}$ is the probability of class $C_k$ generating the term $x_i$
                - $a_k(\mathbf{x})$ are again linear functions of the inputs $x_i$
                - also holds for discrete variables with $M>2$ states
                - App: popular for document classification tasks
        - **Exponential Family**:
            - above results are special cases: If class-conditional densities are members of the exponential family, the posterior $P(C_k\vert\mathbf{x})$ is a logistic sigmoid (2 classses) with argument [4.85]() or the softmax [4.62]() ($K\geq2$ classes) where $a_k$ is given by [4.86](), i.e. again a **generalized linear model**

## probabilistic discriminative models (direct modeling of posterior)

- maximize likelihood function defined through $P(C_k\vert\mathbf{x})$
    - fewer adaptive parameters to be determined than for generative models
        - zB. $M$ parameters for logistic regression vs. $M (M + 5) / 2 + 1$ for Gaussian generative model approach as described above, which grows quadratically with $M$ 
    - may lead to better predictive performance than generative models, particularly when the $P(\mathbf{x}\vert C_k)$ assumptions of the generative models are not accurate
    - Examples:
        - logistic regression (2 classes)
        - softmax regression/multiclass logistic regression (multiple classes)

## generative vs discriminative models

- Generative models:
    - can deal naturally with missing data
    - can handle sequences of varying length (hidden Markov models)
- Discriminative models:
    - generally give better performance on discriminative tasks than generative models
- Combining both: 
    - using kernels: [Bishop_2006](#Bishop_2006) (6.28) ff.
        1. use generative model to define a kernel
        2. use this kernel in a discriminative approach
    - [Laserre_2006](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1640745)

# Linear Basis Function Models<a name="LBF"></a>

- decision boundaries are linear in feature space ($\pmb{\phi}$ space), but nonlinear in input space ($\mathbf{x}$ space)
- Note: nonlinear transformations $\pmb{\phi}$:
    - cannot remove class conditional densities' overlap (i.e. region where posteriors are not $0$ or $1$) ! 
	- they can even increase the level of overlap ! 
	- they can also create overlap where none existed !

# Least Squares and Maximum Likelihood

- S. 141: maximization of likelihood function under a conditional Gaussian (target vector 3.7) noise distribution 3.8 for a linear model is equivalent to minimizing a sum-of-squares error function

# Relation of Gradient Descent to Taylor Expansion

- see [chapter 3.1-3.2](https://www.cs.princeton.edu/courses/archive/fall18/cos597G/lecnotes/lecture3.pdf)

# Newton's method (Newton-Raphson gradient descent)

## Update Formula

$\mathbf{w}^{\tau+1}=\mathbf{w}^{\tau}-\eta\mathbf{H}^{-1}\vec{\nabla}E(\mathbf{w})\vert_{\mathbf{w}^\tau}\text{, where }\mathbf{H}=\nabla\nabla E(\mathbf{w})$

## "local quadratic approximation"

... weil die Newton update Formel $w^{(\tau+1)}=w^{(\tau)}-\ldots$ sich aus der 2nd order Taylor expansion (AKA quadratic approximation) am Punkt $w^\tau$ ergibt, wenn man den 2nd order Polynom nimmt (mit Entwicklungspunkt $w^\tau$), diesen nach $x$ ableitet, $f'(x)=0$ setzt und nach $x$ auflöst. In anderen Worten: Wir suchen die Minimalstelle des 2nd order Taylor Polynoms am Entwicklungspunkt $w^\tau$.

- **Achtung**: die Newton-Raphson method update Formel $w^{(\tau+1)}=w^{(\tau)}-\ldots$ nähert die Nullstelle der 1. Ableitung an und nicht den Funktions**wert**! Letzterer wird über die 2nd order Taylor expansion angenähert!

### 2nd order Taylor polynomial anschaulich

GeoGebra [https://www.geogebra.org/m/tbyAnqAK](https://www.geogebra.org/m/tbyAnqAK)
- beachte: rote Scheitelstelle (vertex Stelle) ist bei -b/(2a), d.h. **nicht** unbedingt bei (0,0) und abhängig von a und b!

Die 2nd order Taylor expansion ist ein Polynom zweiten Grades am Punkt wtau (Parabel, die sich an Stelle wtau an log-likelihood anschmiegt) 
- Entwicklungspunkt wtau muss **nicht** mit Parabel-Scheitelstelle übereinstimmen (s. [Geogebra](https://www.geogebra.org/m/tbyAnqAK))! Diese dürfen auch gar nicht übereinstimmen: Wenn diese übereinstimmen, wäre w(tau) = w(tau + 1), d.h. wtau würde sich nicht verändern beim update: ![Quadratic_approximation](https://i.ibb.co/qk7TNH2/Quadratic-approximation.jpg)
- in 3D: vgl. Bild ganz unten in [Link](https://suzyahyah.github.io/calculus/optimization/2018/04/06/Taylor-Series-Newtons-Method.html) 

## GD vs Newton-Raphson

- Newton's method fails to converge on problems that have **non-differentiable** kinks.  [[Wiki: subgradient methods]](https://en.wikipedia.org/wiki/Subgradient_method)
- For non-differentiable functions use subgradient methods. (see below)

source: [https://www.baeldung.com/cs/gradient-descent-vs-newtons-gradient-descent#newtons-method](https://www.baeldung.com/cs/gradient-descent-vs-newtons-gradient-descent#newtons-method)

<h3 data-id="1-description-of-newtons-method">Description of Newton&#8217;s Method</h3>
<div class="bd-anchor" id="1-description-of-newtons-method"></div>
<p>Newton&#8217;s method works in a different manner [than gradient descent]. <strong>This is because it&#8217;s a method for finding the root of a function, rather than its maxima or minima</strong>.</p>
<p>This means that, if the problem satisfies the constraints of Newton&#8217;s method, we can find <img loading="lazy" src="https://www.baeldung.com/wp-content/ql-cache/quicklatex.com-ede05c264bba0eda080918aaa09c4658_l3.png" class="ql-img-inline-formula quicklatex-auto-format" alt="&#120;" title="Rendered by QuickLaTeX.com" height="8" width="10" style="vertical-align: 0px;" /> for which <img loading="lazy" src="https://www.baeldung.com/wp-content/ql-cache/quicklatex.com-0bce6c022ed0fc63f4659af75888f96c_l3.png" class="ql-img-inline-formula quicklatex-auto-format" alt="&#102;&#40;&#120;&#41;&#61;&#48;" title="Rendered by QuickLaTeX.com" height="19" width="67" style="vertical-align: -5px;" />. Not <img loading="lazy" src="https://www.baeldung.com/wp-content/ql-cache/quicklatex.com-36700780d306ccf4975387990b1949fb_l3.png" class="ql-img-inline-formula quicklatex-auto-format" alt="&#102;&#39;&#40;&#120;&#41;&#61;&#48;" title="Rendered by QuickLaTeX.com" height="19" width="72" style="vertical-align: -5px;" />, as was the case for gradient descent.</p>
<p><strong>We, therefore, apply Newton&#8217;s method on the derivative <img loading="lazy" src="https://www.baeldung.com/wp-content/ql-cache/quicklatex.com-c66e6c51bc42c02f7cfc7a01ce7bdb58_l3.png" class="ql-img-inline-formula quicklatex-auto-format" alt="&#92;&#116;&#101;&#120;&#116;&#98;&#102;&#123;&#102;&#39;&#40;&#120;&#41;&#125;" title="Rendered by QuickLaTeX.com" height="19" width="39" style="vertical-align: -5px;" /> of the cost function, not on the cost function itself</strong>. This is important because Newton&#8217;s method requires the analytical form of the derivative of any input function we use, as we&#8217;ll see shortly. Therefore, <strong>this means that the cost function we use must be differentiable twice</strong>, not just once, as was the case for gradient descent.</p>
<h3 data-id="2-definition">Definition</h3>
<div class="bd-anchor" id="2-definition"></div>
<p>Let&#8217;s define <img loading="lazy" src="https://www.baeldung.com/wp-content/ql-cache/quicklatex.com-a7ee323bc5a3f73ad5e066b13bed5504_l3.png" class="ql-img-inline-formula quicklatex-auto-format" alt="&#102;&#40;&#120;&#41;" title="Rendered by QuickLaTeX.com" height="19" width="34" style="vertical-align: -5px;" /> as the cost function of a model on which we apply Newton&#8217;s method. The terms <img loading="lazy" src="https://www.baeldung.com/wp-content/ql-cache/quicklatex.com-b4caaf19541a3bc05129a71ac72b0bd0_l3.png" class="ql-img-inline-formula quicklatex-auto-format" alt="&#102;&#39;&#40;&#120;&#41;" title="Rendered by QuickLaTeX.com" height="19" width="38" style="vertical-align: -5px;" /> and <img loading="lazy" src="https://www.baeldung.com/wp-content/ql-cache/quicklatex.com-4c8cfa363454f830de83c5485c0f8de0_l3.png" class="ql-img-inline-formula quicklatex-auto-format" alt="&#102;&#39;&#39;&#40;&#120;&#41;" title="Rendered by QuickLaTeX.com" height="19" width="42" style="vertical-align: -5px;" /> thus indicate, respectively, the first and second-order derivatives of <img loading="lazy" src="https://www.baeldung.com/wp-content/ql-cache/quicklatex.com-a7ee323bc5a3f73ad5e066b13bed5504_l3.png" class="ql-img-inline-formula quicklatex-auto-format" alt="&#102;&#40;&#120;&#41;" title="Rendered by QuickLaTeX.com" height="19" width="34" style="vertical-align: -5px;" />. If we start from a point <img loading="lazy" src="https://www.baeldung.com/wp-content/ql-cache/quicklatex.com-2c83758b12d1eb192c053e5f0ac1a434_l3.png" class="ql-img-inline-formula quicklatex-auto-format" alt="&#120;&#95;&#110;" title="Rendered by QuickLaTeX.com" height="11" width="18" style="vertical-align: -3px;" /> that&#8217;s sufficiently close to the minimum of <img loading="lazy" src="https://www.baeldung.com/wp-content/ql-cache/quicklatex.com-9c09a708375fde2676da319bcdfe8b24_l3.png" class="ql-img-inline-formula quicklatex-auto-format" alt="&#102;" title="Rendered by QuickLaTeX.com" height="16" width="10" style="vertical-align: -4px;" />, we can then get a better approximation by computing this formula:</p>
<p style="text-align: center"><img loading="lazy" src="https://www.baeldung.com/wp-content/ql-cache/quicklatex.com-b27a1fd79b3b8caf4ca68b859a4a510f_l3.png" class="ql-img-inline-formula quicklatex-auto-format" alt="&#120;&#95;&#123;&#110;&#43;&#49;&#125;&#32;&#61;&#32;&#120;&#95;&#110;&#32;&#43;&#32;&#92;&#102;&#114;&#97;&#99;&#32;&#123;&#102;&#39;&#40;&#120;&#95;&#110;&#41;&#125;&#32;&#123;&#102;&#39;&#39;&#40;&#120;&#95;&#110;&#41;&#125;" title="Rendered by QuickLaTeX.com" height="29" width="146" style="vertical-align: -10px;" /></p>
<p>The term <img loading="lazy" src="https://www.baeldung.com/wp-content/ql-cache/quicklatex.com-4f53a422f42b582f3c4262da7fc2348c_l3.png" class="ql-img-inline-formula quicklatex-auto-format" alt="&#92;&#102;&#114;&#97;&#99;&#32;&#123;&#102;&#39;&#40;&#120;&#41;&#125;&#32;&#123;&#102;&#39;&#39;&#40;&#120;&#41;&#125;" title="Rendered by QuickLaTeX.com" height="29" width="35" style="vertical-align: -10px;" />, here, indicates that we&#8217;re approximating the function <img loading="lazy" src="https://www.baeldung.com/wp-content/ql-cache/quicklatex.com-b4caaf19541a3bc05129a71ac72b0bd0_l3.png" class="ql-img-inline-formula quicklatex-auto-format" alt="&#102;&#39;&#40;&#120;&#41;" title="Rendered by QuickLaTeX.com" height="19" width="38" style="vertical-align: -5px;" /> with a linear model, in proximity of <img loading="lazy" src="https://www.baeldung.com/wp-content/ql-cache/quicklatex.com-2c83758b12d1eb192c053e5f0ac1a434_l3.png" class="ql-img-inline-formula quicklatex-auto-format" alt="&#120;&#95;&#110;" title="Rendered by QuickLaTeX.com" height="11" width="18" style="vertical-align: -3px;" />.</p>

# Subgradient Methods

- "subgradient methods are convergent when applied even to a **non-differentiable** objective function." [Wiki:Subgradient method](https://en.wikipedia.org/wiki/Subgradient_method)
- Examples:
    - Stochastic Subgradient Descent
- use cases:
    - SVMs (objective: hinge loss)

# Kernel functions

- $k(\mathbf{x},\mathbf{x}^\prime)=\pmb{\phi}(\mathbf{x})^\top\pmb{\phi}(\mathbf{x}^\prime)$
- symmetric function of its arguments $k(\mathbf{x},\mathbf{x}^\prime)=k(\mathbf{x}^\prime,\mathbf{x})$
- Examples:
    - **linear kernels** $k(\mathbf{x},\mathbf{x}^\prime)=\mathbf{x}^\top\mathbf{x}^\prime$
    - **polynomial kernels** $k(\mathbf{x},\mathbf{x}^\prime)=(\mathbf{x}^\top\mathbf{x}^\prime+c)^M$
    - **stationary kernels** $k(\mathbf{x},\mathbf{x}^\prime)=k(\mathbf{x}-\mathbf{x}^\prime)$
        - function only of the difference between the arguments
        - invariant to translations (hence, "stationary")
        - Examples:
            - **homogeneous kernels** = **radial basis functions** $k(\mathbf{x},\mathbf{x}^\prime)=k(||\mathbf{x}-\mathbf{x}^\prime||)$
                - depend only on the magnitude of the distance
                - Examples:
                    - **Gaussian kernels** $k(\mathbf{x},\mathbf{x}^\prime)=\exp(-||\mathbf{x}-\mathbf{x}^\prime||^2/2\sigma^2)$
                        - the feature vector $\pmb{\phi}(\mathbf{x})$ corresponding to the Gaussian kernel has infinite dimensionality!
                        - not restricted to the use of Euclidean distance
    - **sigmoidal kernel/hyperbolic tangent kernel** $k(\mathbf{x},\mathbf{x}^\prime)=\tanh(a\mathbf{x}^\top\mathbf{x}^\prime+b)$
        - **<mark>not a valid kernel!</mark>** 
            - Gramian in general is not positive semidefinite. 
        - However, it has been used in practice (Vapnik, 1995).
- **idea**: if an algorithm is formulated in such a way that the input vector only enters through scalar products, then this scalar product can be replaced with some (other) kernel.
- **necessary and sufficient condition for $k$ to be a valid kernel**: Gram matrix with elements $k(\mathbf{x}_n,\mathbf{x}_m)$ must be positive semidefinite for all possible choices of the set $\{\mathbf{x}_n\}$
    - Note: Not only the Gram matrix but also the kernel itself can be positive-definite, see [Wikipedia](https://en.wikipedia.org/wiki/Positive-definite_kernel)
- one can **construct new kernels** by building them out of simpler kernels, see [Bishop_2006](#Bishop_2006), kernel combination rules (6.13)-(6.22)
- kernel functions can be defined over:
    - graphs
    - sets
    - strings
    - text documents
    - sequences
    - relational data
    - genomic data
    - etc.
- **use cases**:
    - SVMs
    - Kernel PCA
    - Kernel FLD (kernel Fisher discriminant analysis, [KFD](https://en.wikipedia.org/wiki/Kernel_Fisher_discriminant_analysis))
        - "kernelized version of linear discriminant analysis (LDA)" - [Wikipedia](https://en.wikipedia.org/wiki/Kernel_Fisher_discriminant_analysis)

# Linearly separable

- By **<mark>definition</mark>**, the two sets of points $\{\mathbf{x}_n\}$ and $\{\mathbf{y}_n\}$ will be linearly separable if there exists a vector $\mathbf{w}$ and a scalar $w_0$ such that $\mathbf{w}^\top \mathbf{x}_n + w_0 \gt 0$ for all $\mathbf{x}_n$, and $\mathbf{w}^\top \mathbf{y}_n + w_0 \lt 0$ for all $\mathbf{y}_n$. - [Bishop_2006](#Bishop_2006), Exercise 4.1

# SVMs

- **motivation 1**: 
    - **Problem**: 
        - kernel-based algorithms must evaluate the kernel function for all possible pairs of data points
            1. can be computationally infeasible during training
            2. can lead to slow predictions (because of long computing times) at test time
    - **Solution (for ii.)**:
        - **SVMs** only need to evaluate the kernel function at a subset of the training data points (SVMs are kernel-based algorithms that have **sparse** solutions)
- **motivation 2**: 
    - **Problem**:
        - overfitting is often a problem with linearly separable data
            - multiple decision boundaries that have zero error
            - however, they will most likely result in different predictions on the test set, i.e. they will have different generalization performance
                - which one has the **best generalization performance** ?
    - **Solution**:
        - it can be shown that <mark>"the larger the classifier's margin the lower its VC dimension (= capacity for overfitting)"</mark> 
            - $\Rightarrow$ the classifier that has the maximal margin will find the desired decision boundary that has the best generalization performance
                - $\Rightarrow$ the SVM will find the decision boundary with the best generalization performance
- hence, SVMs are robust to "too correct" points !

## Canonical representation of the decision hyperplane

![SVM_canonical_form](/assets/images/SVM_canonical_form.png)

[source](https://ufal.mff.cuni.cz/~hladka/jsmath/test/svm.pdf)

## Soft-margin SVM

- points **inside** the margin are also support vectors! ![slack_variables_SVM](/assets/images/bishop_ml/slack_variables_SVM.png) [[source]](#Bishop_2006)
- for overlapping class distributions
- still sensitive to outliers because the penalty for misclassification increases linearly with $\xi$
- misclassified points have $\xi_n>1$
- $C$ controls the trade-off between the $\xi_n$ penalty and the margin
- $\sum_n\xi_n$ is an **upper bound** on the number of misclassified points
    - because $\xi_n>1$
- Note: kernel trick does **not** avoid curse of dimensionality 
    - because any set of points in the original, say, two-dimensional input space $\mathbf{x}$ would be **constrained** to lie exactly on a two-dimensional **nonlinear manifold** (depending on the kernel) embedded in the higher-dimensional feature space
        - Note: there are **elliptic** and **hyperbolic** paraboloids
- Note: choose monotonically decreasing error function (if the objective is to minimize the misclassification rate)

## Limitations of SVMs

- do not provide posterior probabilistic outputs
    - in contrast, **relevance vector machines (RVM)** do
        - RVMs are based on a Bayesian formulation
        - they have typically **much sparser solutions** than SVMs!

## Training/Solving the Quadratic Programming Problem

- **Training phase** (i.e., the determination of the parameters $\mathbf{a}$ and $b$) makes use of the **whole** data set, as opposed to the test phase, which only makes use of the support vectors
    - **Problem:** Direct solution is often infeasible (demanding computation and memory requirements)
- $\tilde{L}(\mathbf{a})$ is quadratic
    - linear constraints $\Rightarrow$ constraints define a convex region $\Rightarrow$ any local optimum will be a global optimum
- Most popular approach to training SVMs: **Sequential minimal optimization (SMO)**
    - SMO uses an extreme form of the **chunking** method 
    - standard chunking method:
        - i.e., identifies <mark>**all**</mark> of the nonzero Lagrange multipliers and discards the others
            - works because Lagrangian does not change, if the rows and columns of the kernel matrix corresponding to Lagrange multipliers that have value zero are removed 
                - $\Rightarrow$ size of the matrix in the quadratic function is reduced from $N^2$ to $\left(\#(\text{nonzero Lagrange multipliers})\right)^2$, where $N$ is the number of data points
                    - **Problem:** still needs too much memory for large-scale applications
    - SMO considers just <mark>**two**</mark> Lagrange multipliers at a time, so that the subproblem can be solved analytically instead of numerically
        - those two Lagrange multipliers are chosen using heuristics at each step 
    - SMO scaling in practice: between $\mathcal{O}(N)$ and $\mathcal{O}(N^2)$

## Probabilistic Outputs

- SVM does not provide probabilistic outputs
- [Platt_2000](https://www.researchgate.net/publication/2594015_Probabilistic_Outputs_for_Support_Vector_Machines_and_Comparisons_to_Regularized_Likelihood_Methods) has proposed fitting a logistic sigmoid to the outputs of a **trained** SVM
    - **Problem:** SVM training process is not geared towards this
        - therefore, the SVM can give poor approximations to the posteriors

## Apps

- Text Classification
    - "Spam or Ham", spam filter
        - "Bag-of-words" approach
        - Histogram of word counts
            - very **high-dimensional** feature space ($\approx 10000$ dimensions)
            - few irrelevant features
- OCR, Handwritten digit recognition
    - Virtual SVM (V-SVM), Schölkopf, **0.8% error rate on MNIST**
        - LeNet-1, 1.7% error rate
        - LeNet-5, 0.95% error rate
    - comparison: [overview](/assets/images/LeNetvsSVM.png), see [LeNet-5 paper](http://yann.lecun.com/exdb/publis/pdf/lecun-01a.pdf)
    - Note: LeNet in general refers to LeNet-5
    - <mark>SVMs show **almost no overfitting** with higher-degree kernels!</mark>
- Object Detection
    - SVMs are **real-time capable**
        - sliding-window approach
            - **idea**: classify "object vs non-object" for each window
                - e.g. pedestrian vs non-pedestrian
- High-energy physics
- protein secondary structure prediction
- etc.

# Neural Networks

## Perceptrons (Rosenblatt 1962)

- perceptrons are **generalized linear models** ("generalized" because of the activation function)
    - **BUT**: Deep Neural Networks are **nonlinear parametric models**.
- more specifically: perceptrons are **generalized linear discriminants** (because they map the input **x** directly to a class label t in {-1,+1} [see above: "Linear models for classification": approach 1.])
- original version: 
    - 2-class linear discriminant 
    - with fixed [i.e. not learned!] nonlinear transformation $\vec{\phi}(\pmb{x})$
    - activation function: step function
    - learned via minimization of "**perceptron criterion**" $\Rightarrow$ SGD
    - exact solution guaranteed for linearly separable data set (**Perceptron Convergence Theorem**)
        - **BUT:** in practice, convergence can be slow
            - it's hard to decide, if a problem is not linearly separable or just slowly converging!

## Terminology

- **Input layer** is a layer, it's not wrong to say that. [source](https://datascience.stackexchange.com/a/14033/115254)

- However, when calculating the **depth** of a deep neural network, we only consider the layers that have tunable weights. [source](https://datascience.stackexchange.com/a/14033/115254)

## Automatic Differentiation

### Forward-mode vs Reverse-mode differentiation

- read [Olah](https://colah.github.io/posts/2015-08-Backprop/)

> **Forward-mode differentiation** starts at an input to the graph and moves towards the end. At every node, it sums all the paths feeding in. Each of those paths represents one way in which the input affects that node. By adding them up, we get the total way in which the node is affected by the input, it’s derivative. [...]

> **Reverse-mode differentiation**, on the other hand, starts at an output of the graph and moves towards the beginning. At each node, it merges all paths which originated at that node. [...]

> When I say that reverse-mode differentiation gives us the derivative of e with respect to every node, I really do mean **every node**. We get both $\frac{\partial e}{\partial a}$ and $\frac{\partial e}{\partial b}$, the derivatives of $e$ with respect to both inputs. Forward-mode differentiation gave us the derivative of our output with respect to a single input, but reverse-mode differentiation gives us all of them. [...] 

> When training neural networks, we think of the cost (a value describing how bad a neural network performs) as a function of the parameters (numbers describing how the network behaves). We want to calculate the derivatives of the **cost with respect to all the parameters**, for use in gradient descent. Now, there’s often millions, or even tens of millions of parameters in a neural network. So, **reverse-mode differentiation, <mark>called</mark> backpropagation** [[more precise: reverse_mode_accumulation](#reverse_mode_accumulation)] in the context of neural networks, gives us a massive speed up!

> (Are there any cases **where forward-mode differentiation makes more sense**? Yes, there are! Where the reverse-mode gives the derivatives of one output with respect to all inputs, the forward-mode gives us the derivatives of all outputs with respect to one input. If one has a function with lots of outputs, forward-mode differentiation can be much, much, much faster.) 

- both are algorithms for efficiently computing the sum by factoring the paths. Instead of summing over all of the paths explicitly, they compute the same sum more efficiently by <mark>**merging paths back together at every node**</mark>. In fact, **both** algorithms touch each edge exactly once!
    - At each node, reverse-mode differentiation merges all paths which **originated** at that node (starting at an output of the graph and moving towards the beginning)
    - At each node, forward-mode differentiation sums all the paths **feeding into** that node (starting at the beginning and moving towards an output of the graph)
- forward-mode: apply operator $\frac{\partial}{\partial X}$ 
- reverse-mode: apply operator $\frac{\partial Z}{\partial}$
- if we have e.g. a hundred inputs, but only one output, reverse-mode differentiation gives a speed up in $\mathcal{O}(\text{\# Inputs})$ compared to forward-mode differentiation

### PyTorch autograd

[source: Justin Johnson](https://pytorch.org/tutorials/beginner/pytorch_with_examples.html) 

- In the above examples, we had to **manually** implement both the forward and backward passes of our neural network. Manually implementing the backward pass is not a big deal for a small two-layer (?: siehe Stichpunkt) network, but can quickly get very hairy for large complex networks.
    - ?: Why "two-layer": 
        - The previous polynomial regression examples correspond to a **single** layer perceptron with a fixed nonlinear transformation of the inputs (here: using polynomial basis functions), so why does Johnson say **two**-layer perceptron?
            - What Johnson probably means here is that, basically, implementing backprop **manually** (like in the previous polynomial regression examples) for a two-layer NN would be possible without autograd. This "two-layer network", however, does not refer to the previous polynomial regression models!
- `autograd` computes **all** gradients with only one line `loss.backward()`.
    - in polynomial regression example **without** `autograd`:
        ```python
        grad_a = grad_y_pred.sum()
        grad_b = (grad_y_pred * x).sum()
        grad_c = (grad_y_pred * x ** 2).sum()
        grad_d = (grad_y_pred * x ** 3).sum()
        ```
    - the same **with** `autograd`:
        ```python
        loss.backward()
        ```
        where all parameter tensors must have `requires_grad = True` (otherwise `autograd` does not know wrt which parameters `loss` must be differentiated).
- Thankfully, we can use **automatic differentiation** to automate the computation of backward passes in neural networks. The **autograd** package in PyTorch provides exactly this functionality. When using autograd, the forward pass of your network will define a **computational graph**; nodes in the graph will be Tensors, and edges will be functions that produce output Tensors from input Tensors. Backpropagating through this graph then allows you to easily compute gradients.
	- auf Folie:
		1. Convert NN to a computational graph 
			- explanations:
				- [PyTorch 101, Part 1: Understanding Graphs, Automatic Differentiation and Autograd](https://blog.paperspace.com/pytorch-101-understanding-graphs-and-automatic-differentiation/)
					- [important points from this blog post](/pytorch/machine_learning/notes-pytorch/#how-does-pytorch-create-a-computational-graph)
				- [Computational graphs in PyTorch and TensorFlow](https://towardsdatascience.com/computational-graphs-in-pytorch-and-tensorflow-c25cc40bdcd1)
		2. Each new layer/module specifies how it affects the forward and backward passes 
			- auf nächster Folie: "Each module is defined by
				- `module.fprop(`$x$`)`
				- `module.bprop(`$\frac{\partial E}{\partial y}$`)`
					- computes the gradients of the cost wrt. the inputs $x$ given the gradient wrt. the outputs $y$
					- `module.bprop()` ist in PyTorch wegen dem Autograd System nicht notwendig (vgl. [aus PyTorch Doc](/pytorch/machine_learning/notes-pytorch/#modules))
			- e.g. `torch.nn.Linear` specifies that it will apply a linear transformation $y=xA^T+b$ to the incoming data during the forward pass (each module has a `forward()` method, see e.g. [source nn.Linear](https://pytorch.org/docs/stable/_modules/torch/nn/modules/linear.html#Linear))
		3. Apply reverse-mode differentiation 
			- i.e. call `loss.backward()`

- This sounds complicated, it’s pretty simple to use in practice. Each Tensor represents a node in a computational graph. If `x` is a Tensor that has `x.requires_grad=True` then `x.grad` is another Tensor holding the gradient of `x` with respect to some scalar value.

```python
# -*- coding: utf-8 -*-
import torch
import math


# Create Tensors to hold input and outputs.
x = torch.linspace(-math.pi, math.pi, 2000)
y = torch.sin(x)

# For this example, the output y is a linear function of (x, x^2, x^3), so
# we can consider it as a linear layer neural network. Let's prepare the
# tensor (x, x^2, x^3).
p = torch.tensor([1, 2, 3])
xx = x.unsqueeze(-1).pow(p)

# In the above code, x.unsqueeze(-1) has shape (2000, 1), and p has shape
# (3,), for this case, broadcasting semantics will apply to obtain a tensor
# of shape (2000, 3)

# Use the nn package to define our model as a sequence of layers. nn.Sequential
# is a Module which contains other Modules, and applies them in sequence to
# produce its output. The Linear Module computes output from input using a
# linear function, and holds internal Tensors for its weight and bias.
# The Flatten layer flatens the output of the linear layer to a 1D tensor,
# to match the shape of `y`.
model = torch.nn.Sequential(
    torch.nn.Linear(3, 1),
    torch.nn.Flatten(0, 1)
)

# The nn package also contains definitions of popular loss functions; in this
# case we will use Mean Squared Error (MSE) as our loss function.
loss_fn = torch.nn.MSELoss(reduction='sum')

learning_rate = 1e-6
for t in range(2000):

    # Forward pass: compute predicted y by passing x to the model. Module objects
    # override the __call__ operator so you can call them like functions. When
    # doing so you pass a Tensor of input data to the Module and it produces
    # a Tensor of output data.
    y_pred = model(xx)

    # Compute and print loss. We pass Tensors containing the predicted and true
    # values of y, and the loss function returns a Tensor containing the
    # loss.
    loss = loss_fn(y_pred, y)
    if t % 100 == 99:
        print(t, loss.item())

    # Zero the gradients before running the backward pass.
    model.zero_grad()

    # Backward pass: compute gradient of the loss with respect to all the learnable
    # parameters of the model. Internally, the parameters of each Module are stored
    # in Tensors with requires_grad=True, so this call will compute gradients for
    # all learnable parameters in the model.
    loss.backward()

    # Update the weights using gradient descent. Each parameter is a Tensor, so
    # we can access its gradients like we did before.
    with torch.no_grad():
        for param in model.parameters():
            param -= learning_rate * param.grad

# You can access the first layer of `model` like accessing the first item of a list
linear_layer = model[0]

# For linear layer, its parameters are stored as `weight` and `bias`.
print(f'Result: y = {linear_layer.bias.item()} + {linear_layer.weight[:, 0].item()} x + {linear_layer.weight[:, 1].item()} x^2 + {linear_layer.weight[:, 2].item()} x^3')
```

## Forward Propagation

- inputs:
	- depth $l$
	- $l$ weight matrices of the model $\mathbf{W}^{(i)}$
	- $l$ biases of the model $\mathbf{b}^{(i)}$
	- input $\mathbf{x}$ (here: only one for simplicity)
	- target $\mathbf{y}$
- outputs:
	- output $\hat{\mathbf{y}}$
	- cost function $J$
	- input of unit $j$: $\mathbf{a}_j^{(k)}$ for all $j$
	- output of unit $j$: $\mathbf{h}_j^{(k)}$ for all $j$
 
## Backprop

- inputs:
	- depth l
	- l weight matrices of the model $\mathbf{W}^{(i)}$
	- l biases of the model $\mathbf{b}^{(i)}$
	- outputs of Forward Propagation
- outputs:
	- gradients w.r.t. all weights and biases $\nabla_{\mathbf{W}^{(k)}}J$ and $\nabla_{\mathbf{b}^{(k)}}J$
		- also computes all $\nabla_{\mathbf{a}^{(k)}}J$ and $\nabla_{\mathbf{h}^{(k)}}J$ in the process
			- $\nabla_{\mathbf{a}^{(k)}}J$ can be interpreted as an indication of how each layer’s output should change to reduce error
				- es gibt ein $\nabla_{\mathbf{a}^{(k)}}J$ pro layer k: jede unit in layer k entspricht einer Komponente von $\nabla_{\mathbf{a}^{(k)}}J$

- refers only to the **method used to compute all necessary gradients**, whereas another algorithm (e.g. SGD) is used to perform **learning** using these gradients!
	- "however, the term is often used loosely to refer to the entire learning algorithm, including how the gradient is used, such as by stochastic gradient descent" [source](https://en.wikipedia.org/wiki/Backpropagation)
       	> <a name="reverse_mode_accumulation"></a>"More generally, the field of **automatic differentiation** is concerned with how to compute derivatives algorithmically. The back-propagation algorithm described here is only one approach to automatic differentiation. It is a special case of a broader class of techniques called **reverse mode accumulation**." (Goodfellow, Bengio)

- "layer below builds upon (gradient) result of layer above" (basically, chain rule)
	- this is why it's called "backprop"
	- "propagates the gradient backwards through the layers"
- "performs on the order of one **Jacobian product** per node in the graph" (Goodfellow, Bengio)
    - This can be seen from the fact that Backprop visits each edge (of the computational graph for this problem) only once 
- "[...] the amount of computation required for performing the back-propagation **scales linearly with the number of edges** in $\mathcal{G}$, where the computation **for each edge** corresponds to computing 
    - a partial derivative (of one node with respect to one of its parents) as well as performing 
    - one multiplication and 
    - one addition." (Goodfellow, Bengio)

### Computational Graphs

- the following texts from [Goodfellow_2016](#Goodfellow_2016) describe the same graphs as Olah is describing in his [blog post](https://colah.github.io/posts/2015-08-Backprop/)
    - "That algorithm specifies the **forward propagation** computation, which we could put in a graph $\mathcal{G}$. In order to perform **back-propagation**, we can construct a computational graph that depends on $\mathcal{G}$ and adds to it an extra set of nodes. These form a **subgraph** $\mathcal{B}$ with one node per node of $\mathcal{G}$. Computation in $\mathcal{B}$ proceeds in exactly the reverse of the order of computation in $\mathcal{B}$, and each node of $\mathcal{B}$ computes the derivative $\frac{\partial u^{(n)}}{\partial u^{(i)}}$ associated with the **forward graph** node $u^{(i)}$." (Goodfellow, Bengio)
    - "The subgraph $\mathcal{B}$ contains exactly one edge for each edge from node $u^{(j)}$ to node $u^{(i)}$ of $\mathcal{G}$." (Goodfellow, Bengio)

### Dynamic Programming

- a computer programming method
	- though, in literature one often finds the plural form "dynamic programming methods"
- refers to simplifying a complicated problem by breaking it down into simpler sub-problems in a recursive manner
	- if this "breaking down" is possible for a problem, then the problem is said to have **optimal substructure**

#### Example: Fibonacci sequence

source: [https://en.wikipedia.org/wiki/Dynamic_programming#Fibonacci_sequence](https://en.wikipedia.org/wiki/Dynamic_programming#Fibonacci_sequence)

```python
var m := map(0 → 0, 1 → 1)
function fib(n)
    if key n is not in map m
        m[n] := fib(n − 1) + fib(n − 2)
    return m[n]
```

- This technique of saving values that have already been calculated is called **memoization**
- The function requires only $\mathcal{O}(n)$ time instead of **exponential time** (but requires $\mathcal{O}(n)$ space)
    - i.e. the number of common subexpressions is reduced **without regard to memory**!
	- note: sometimes recalculating instead of storing can be a good decision, **if memory is limited**!

#### Relation to Backprop

- Backprop stores the $y_i^{(k-1)}$ during the forward pass and re-uses it during the backward pass to calculate $\frac{\partial E}{\partial w_{ji}^{(k-1)}}=y_i^{(k-1)}\frac{\partial E}{\partial w_{ji}^{(k-1)}}$ (memoization, Dynamic Programming)
- During the backward pass Backprop visits each edge only once (see above) and gradients that have already been calculated are saved in memory (cf. `grad_table[u[i]]` in Algo 6.2 or `g` in Algo 6.4 Goodfellow, Bengio)! (memoization, Dynamic Programming)
    - this is analogous to the Fibonacci Sequence Algo's map `m` (see above) which saves the `fib(n − 1) + fib(n − 2)` that have already been calculated in memory
- (cf. Figure 6.9 in Goodfellow, Bengio) Back-propagation avoids the exponential explosion in **repeated subexpressions** 
- similar to the Fibonacci example "the back-propagation algorithm is designed to reduce the number of common subexpressions **without regard to memory**." (Goodfellow, Bengio)
- "When the memory required to store the value of these expressions is low, the back-propagation approach of equation 6.52 ![6.52](/assets/images/goodfellow_ml/Goodf_6_50-6_53.png) is clearly preferable because of its reduced runtime. However, equation 6.53 is also a valid implementation of the chain rule, and is useful **when memory is limited**." (Goodfellow, Bengio)

## Implementing Softmax Correctly

- Problem: Exponentials get very big and can have very different magnitudes
	- Solution: 
		- Evaluate $\ln{(\sum_{j=1}^K\exp{(\mathbf{w}_j^\top\mathbf{x})})}$ in the denominator **before** calculating the fraction
		- since $\text{softmax}(\mathbf{a} + \mathbf{b}) = \text{softmax}(\mathbf{a})$ for all $\mathbf{b}\in\mathbb{R}^D$, one can subtract the largest $\mathbf{w}_j$ from the others
            - (entspricht $\mathbf{a}=\mathbf{w}_j^\top\mathbf{x}$ und $\mathbf{b}=\mathbf{w}_M^\top\mathbf{x}$ bzw. Kürzen des Bruches mit $\exp{(\mathbf{w}_M^\top\mathbf{x})}$, wobei $\mathbf{w}_M$ das größte weight ist)
            - (egal, ob $\mathbf{b}$ von $\mathbf{x}$ abhängt oder nicht!)

## MLP in numpy from scratch

- see [here](https://htmlpreview.github.io/?https://github.com/pharath/home/blob/master/_posts_html/notebooks_in_html/Expl_NN_in_numpy_copy.html)

## Stochastic Learning vs Batch Learning

source: LeCun et al. "Efficient BackProp"

### SGD

- Pros:
	- is usually much faster than batch learning
		- consider large redundant data set
			- example: training set of size 1000 is inadvertently composed of 10 identical copies of a set with 100 samples
	- also often results in better solutions because of the noise in the updates
		- because the noise present in the updates can result in the weights jumping into the basin of another, possibly deeper, local minimum. This has been demonstrated in certain simplified cases
	- can be used for tracking changes
		-  useful when the function being modeled is changing over time

- Cons:
	- noise also prevents full convergence to the minimum 
		- Instead of converging to the exact minimum, the convergence stalls out due to the **weight fluctuations** 
		- size of the fluctuations depend on the degree of noise of the stochastic updates:
			- The variance of the fluctuations around the local minimum is proportional to the learning rate $\eta$ 
			- So in order **to reduce the fluctuations** we can either 
				- decrease (anneal) the learning rate or
				- have an adaptive batch size.

### Batch GD

- Pros:
	- Conditions of convergence are well understood.
	- Many acceleration techniques (e.g. conjugate gradient) only operate in batch learning.
     	- Theoretical analysis of the weight dynamics and convergence rates are simpler
	- one is able to use second order methods to speed the learning process 
		- Second order methods speed learning by estimating not just the gradient but also the curvature of the cost surface. Given the curvature, one can estimate the approximate location of the actual minimum.

- Cons:
	- redundancy can make batch learning much slower than on-line
	- often results in worse solutions because of the absence of noise in the updates
		- will discover the minimum of whatever basin the weights are initially placed
	- changes go undetected and we obtain rather bad results since we are likely to average over several rules

### Mini-batch GD

- Another method to remove noise [in SGD] is to use “mini-batches”, that is, start with a small batch size and increase the size as training proceeds. 
	- However, deciding the rate at which to increase the batch size and which inputs to include in the small batches is as difficult as determining the proper learning rate. **Effectively the size of the learning rate in stochastic learning corresponds to the respective size of the mini batch.**
- Note also that the problem of removing the noise in the data may be less critical than one thinks because of generalization. **Overtraining may occur long before the noise regime is even reached.**

### Shuffling the Examples

- [Expl_NN_in_numpy](https://nbviewer.org/github/pharath/home/blob/master/assets/notebooks/Expl_NN_in_numpy.ipynb)
- [MLP_in_numpy](https://nbviewer.org/github/pharath/home/blob/master/assets/notebooks/MLP_in_numpy.ipynb)
- [MLP_selbst_versucht](https://nbviewer.org/github/pharath/home/blob/master/assets/notebooks/MLP_selbst_versucht.ipynb)
- [WofuerIst__name__gut](https://nbviewer.org/github/pharath/home/blob/master/assets/notebooks/WofuerIst__name__gut.ipynb)
     
# REFERENCES

- <a name="Bishop_2006"></a> [Bishop, Christopher M., *Pattern Recognition and Machine Learning (Information Science and Statistics)* (2006), Springer-Verlag, Berlin, Heidelberg, 0387310738.][1]
- <a name="Goodfellow_2016"></a> [Ian J. Goodfellow and Yoshua Bengio and Aaron Courville, *Deep Learning* (2016), MIT Press, Cambridge, MA, USA][2]

[1]: https://www.amazon.de/Pattern-Recognition-Learning-Information-Statistics/dp/0387310738
[2]: http://www.deeplearningbook.org
