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
> - generalized linear models

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
	- p(Ck|x) can be written as logistic sigmoid 4.57 
		- (i.e. it has a sigmoidal shape in input space, **if** "a" [4.58] is linear in x!)
	- first model p(x|Ck) and p(Ck), then use 4.57-58 to find p(Ck|x) (or use equivalently Bayes' theorem 1.82-83)
		- Examples:
			- model p(x|Ck) as Gaussian 4.64 **=>** posterior p(Ck|x) is the logistic sigmoid 4.65, i.e. a **generalized linear model**
				- (i.e. linear decision boundaries, but not linear in parameters!)
				- decision boundaries are where (the 2 largest) posteriors are equal
				- use MaxLike to determine parameters of Gaussian 4.64 and priors p(Ck) (requires data set)
3. discriminative models (direct modeling of posterior)
