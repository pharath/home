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
