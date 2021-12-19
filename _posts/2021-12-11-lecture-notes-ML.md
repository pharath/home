---
title: "Machine Learning"
excerpt: "Page formatting sucks. Click on \"Content in HTML\" button below."
header:
  teaser: /assets/images/databases.png
  overlay_image: /assets/images/databases.png
  overlay_filter: 0.5 
  caption: "Photo credit: [**Jennifer Widom**](https://cs.stanford.edu/people/widom/)"
  actions:
    - label: "Content in HTML"
[//]: # (      url: "https://htmlpreview.github.io/?https://github.com/pharath/home/blob/master/_posts_html/2021-09-23-Databases.html")
categories:
  - Lecture_Notes
  - Machine_Learning
tags:
  - lecture_notes
  - ml
toc: true
toc_sticky: true
last_modified_at: 2021-09-23T16:00:52-04:00

---

# Overfitting problem

## Paradoxes

1. higher order polynomial contains all lower order polynomials as special cases
	- hence, the higher order polynomial **is** capable of generating a result at least as good as the lower order polynomial
		- d.h. das Polynom mit höherer Ordnung **kann** potentiell mindestens genauso gute Ergebnisse liefern ! Wieso passiert das nicht ?
2. Sinus-Potenzreihe konvergiert für alle x in R und enthält Terme mit allen Potenzen von x (die geraden Potenzen haben Vorfaktor 0)

Aus 1. und 2. folgt, dass das Ergebnis mit jedem weiteren höheren x-Term umso genauer (d.h. näher an der ursprünglichen Sinusfunktion) werden sollte (weil die Potenzreihe ja auch mit jedem höheren x-Term genauer wird). Warum passiert das nicht ? 

**Antwort**:

Bei der Potenzreihe (Maclaurin Series) bleiben die weights der ersten N Terme fix, wenn man den Term der Ordnung N+1 hinzufügt, aber beim Fitten nicht ! Die Weights werden beim overfitting zu groß und sorgen für eine schlechtere representation der Targetfunktion als die Potenzreihenrepresentation der Targetfunktion, die gleiche Ordnung hat!

Deshalb muss man große weights "bestrafen" (d.h. einen penalty term einfügen aka regularizen), um die weights beim Fitten klein zu halten.

Wir geben ja den weights völlige Freiheit, aber genau das ist das Problem: die weights fangen beim overfitting an auch den Noise in den Daten zu fitten, was für die hohen weight Werte sorgt! 

## Possible Solutions

- reduce number of model parameters
- increase number of data points N
- increase regularization parameter lambda (in statistics: shrinkage methods)
    - quadratic: 
        - weight decay (in Deep Learning)
        - L2 regularization = ridge regression (in statistics)

# Probability Theory

- Let p(B=r) = 0.4 and p(B=b) = 0.6 (frequentist view: this is how many times the guy who picked the fruit picked each box (in the limit of infinitely many pickings), i.e. the picker had a tendency to pick the blue one more often). 
	- Suppose that we pick a box at random, and it turns out to be the blue box. What's the probability of picking an apple ?
	- What's the (overall) probability of choosing an apple ?
	- Suppose we picked an orange. Which box did it come from ?
- prior vs posterior explain intuitively (e.g. "evidence favouring the red box")
- independence intuitively (e.g. "independent of which box is chosen")

# Outlier removal

- by choosing a suitable nonlinearity (e.g. sigmoid, tanh)
- remove outlier data point completely from the data set

# Linear models for classification

> Note:
> - linear models (i.e. WITHOUT activation function) 
> - generalized linear models (WITH activation function)

1. linear discriminant functions
	- 2 classes
	- K classes
		- one-vs-rest
		- one-vs-one
		- single K-class discriminant
	- learning the parameters
		- least squares
		- Fisher's linear discriminant
		- perceptron algorithm
2. probabilistic generative models (indirect modeling of posterior)
	- p(Ck\|x) can be written as logistic sigmoid 4.57 
		- (i.e. it has a sigmoidal shape in input space, **if** "a" [4.58] is linear in x!)
	- first model p(x\|Ck) and p(Ck), then use 4.57-58 to find p(Ck\|x) (or use equivalently Bayes' theorem 1.82-83)
		- Examples:
			- model p(x\|Ck) as Gaussian 4.64 **=>** posterior p(Ck\|x) is the logistic sigmoid 4.65, i.e. a **generalized linear model**
				- (i.e. linear decision boundaries, but not linear in parameters!)
				- decision boundaries are where (the 2 largest) posteriors are equal
				- use MaxLike to determine parameters of Gaussian 4.64 and priors p(Ck) (requires data set)
				- priors enter only through w0 4.67
					- i.e. priors shift the decision boundary parallelly (vgl. 4.65 mit distance from the origin to the decision surface 4.5)
					- i.e. priors shift the parallel contours of constant posterior probability
3. probabilistic discriminative models (direct modeling of posterior)
	- maximize likelihood function defined through p(Ck\|x)
		- fewer adaptive parameters to be determined than for generative models
			- zB. M parameters for logistic regression vs. M (M + 5) / 2 + 1 for Gaussian generative model approach as described above, which grows quadratically with M ! 
		- may lead to better predictive performance than generative models, particularly when the p(x\|Ck) assumptions of the generative models are not accurate
		- Examples:
			- logistic regression (2 classes)
			- softmax regression/multiclass logistic regression (multiple classes)

# Linear Basis Function Models

- decision boundaries are linear in feature space (phi space), but nonlinear in input space (x space)
- nonlinear transformations phi cannot remove class conditional densities' overlap (i.e. region where posteriors are not 0 or 1) ! 
	- they can even increase the level of overlap ! 
	- they can also create overlap where none existed !

# Least Squares and Maximum Likelihood

- S. 141: maximization of likelihood function under a conditional Gaussian (target vector 3.7) noise distribution 3.8 for a linear model is equivalent to minimizing a sum-of-squares error function

# Newton's method (Newton-Raphson gradient descent)

## "local quadratic approximation"

... weil die Newton update Formel [w(tau+1)=w(tau)-...] sich aus der 2nd order Taylor expansion (AKA quadratic approximation) am Punkt wtau ergibt, wenn man den 2nd order Polynom nimmt (mit Entwicklungspunkt wtau), diesen nach x ableitet, f'(x)=0 setzt und nach x auflöst. In anderen Worten: Wir suchen die Minimalstelle des 2nd order Taylor Polynoms am Entwicklungspunkt wtau.

- **Achtung**: die Newton-Raphson method update Formel [w(tau+1)=w(tau)-...] nähert die Nullstelle der 1. Ableitung an und nicht den Funktions**wert**! Letzterer wird über die 2nd order Taylor expansion angenähert!

### 2nd order Taylor polynomial anschaulich

GeoGebra [https://www.geogebra.org/m/tbyAnqAK](https://www.geogebra.org/m/tbyAnqAK)
- beachte: rote Scheitelstelle (vertex Stelle) ist bei -b/(2a), d.h. **nicht** unbedingt bei (0,0) und abhängig von a und b!

Die 2nd order Taylor expansion ist ein Polynom zweiten Grades am Punkt wtau (Parabel, die sich an Stelle wtau an log-likelihood anschmiegt) 
- Entwicklungspunkt wtau muss **nicht** mit Parabel-Scheitelstelle übereinstimmen (s. [Geogebra](https://www.geogebra.org/m/tbyAnqAK))! Diese dürfen auch gar nicht übereinstimmen: Wenn diese übereinstimmen, wäre w(tau) = w(tau + 1), d.h. wtau würde sich nicht verändern beim update: ![Quadratic_approximation](https://i.ibb.co/qk7TNH2/Quadratic-approximation.jpg)
- in 3D: vgl. Bild ganz unten in [Link](https://suzyahyah.github.io/calculus/optimization/2018/04/06/Taylor-Series-Newtons-Method.html) 

## GD vs Newton-Raphson

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

# Neural Networks

## Perceptrons (Rosenblatt 1962)

- perceptrons are **generalized linear models** ("generalized" because of the activation function)
- more specifically: perceptrons are **generalized linear discriminants** (because they map the input **x** directly to a class label t in {-1,+1} [see above: "Linear models for classification": approach 1.])

## Terminology

- **Input layer** is a layer, it's not wrong to say that. [source](https://datascience.stackexchange.com/a/14033/115254)

- However, when calculating the **depth** of a deep neural network, we only consider the layers that have tunable weights. [source](https://datascience.stackexchange.com/a/14033/115254)

## Forward-mode vs Reverse-mode differentiation

> When I say that reverse-mode differentiation gives us the derivative of e with respect to every node, I really do mean **every node**. We get both $\frac{\partial e}{\partial a}$ and $\frac{\partial e}{\partial b}$, the derivatives of $e$ with respect to both inputs. Forward-mode differentiation gave us the derivative of our output with respect to a single input, but reverse-mode differentiation gives us all of them. [...] 
> Are there any cases where forward-mode differentiation makes more sense? Yes, there are! Where the reverse-mode gives the derivatives of one output with respect to all inputs, the forward-mode gives us the derivatives of all outputs with respect to one input. If one has a function with lots of outputs, forward-mode differentiation can be much, much, much faster.

## Forward Propagation

- inputs:
	- depth l
	- l weight matrices of the model $\mathbf{W}^{(i)}$
	- l biases of the model $\mathbf{b}^{(i)}$
	- input $\mathbf{x}$ (here: only one for simplicity)
	- target $\mathbf{y}$
- outputs:
	- output $\hat{\mathbf{y}}$
	- cost function $J$
	- input of unit j: $\mathbf{a}_j^{(k)}$ for all j
	- output of unit j: $\mathbf{h}_j^{(k)}$ for all j

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
       	> "More generally, the field of **automatic differentiation** is concerned with how to compute derivatives algorithmically. The back-propagation algorithm described here is only one approach to automatic differentiation. It is a special case of a broader class of techniques called **reverse mode accumulation**." (Goodfellow, Bengio)

- "layer below builds upon (gradient) result of layer above" (basically, chain rule)
	- this is why it's called "backprop"
	- "propagates the gradient backwards through the layers"
- "performs on the order of one **Jacobian product** per node in the graph" (Goodfellow, Bengio)
- Backprop visits each edge (of the computational graph for this problem) only once, so that "[...] the amount of computation required for performing the back-propagation **scales linearly with the number of edges** in G, where the computation for each edge corresponds to computing a partial derivative (of one node with respect to one of its parents) as well as performing one multiplication and one addition." (Goodfellow, Bengio)

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
- The function requires only O(n) time instead of exponential time (but requires O(n) space)
    - i.e. the number of common subexpressions is reduced **without regard to memory**!
	- note: sometimes recalculating instead of storing can be a good decision, if memory is limited!

#### Relation to Backprop

- Backprop stores the $y_i^{(k-1)}$ during the forward pass and re-uses it during the backward pass to calculate $$\frac{\partial E}{\partial w_{ji}^{(k-1)}}=y_i^{(k-1)}\frac{\partial E}{\partial w_{ji}^{(k-1)}}$$ (memoization, Dynamic Programming)
- During the backward pass Backprop visits each edge only once (see above) and gradients that have already been calculated are saved in memory (cf. `grad_table[u[i]]` in Algo 6.2 or `g` in Algo 6.4 Goodfellow, Bengio)! (memoization, Dynamic Programming)
    - this is analogous to the Fibonacci Sequence Algo's map `m` (see above) which saves the `fib(n − 1) + fib(n − 2)` that have already been calculated in memory
- (cf. Figure 6.9 in Goodfellow, Bengio) Back-propagation avoids the exponential explosion in **repeated subexpressions** 
- similar to the Fibonacci example "the back-propagation algorithm is designed to reduce the number of common subexpressions **without regard to memory**." (Goodfellow, Bengio)
- "When the memory required to store the value of these expressions is low, the back-propagation approach of equation 6.52 is clearly preferable because of its reduced runtime. However, equation 6.53 is also a valid implementation of the chain rule, and is useful **when memory is limited**." (Goodfellow, Bengio)
