---
title: "PyTorch"
excerpt: "Notes on PyTorch."
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
  - PyTorch
  - Machine_Learning
tags:
  - pytorch
  - ml
toc: true
toc_label: "Contents"
last_modified_at: 2021-09-23T16:00:52-04:00

---

# PyTorch Doc

[source](https://pytorch.org/docs/stable/notes/modules.html)

## Modules

- read [A Simple Custom Module](https://pytorch.org/docs/stable/notes/modules.html#a-simple-custom-module)
- "Note that the module itself is callable, and that calling it invokes its `forward()` function. This name is in reference to the concepts of “forward pass” and “backward pass”, which apply to each module. 
	- The <mark>“forward pass”</mark> is responsible for applying the computation represented by the module to the given input(s) (as shown in the above snippet). 
	- The <mark>“backward pass”</mark> **computes gradients** of module outputs with respect to its inputs, which can be used for “training” parameters through gradient descent methods. 
        - PyTorch’s **autograd system** automatically takes care of this backward pass computation, so it is not required to manually implement a `backward()` function for each module."

# How does PyTorch create a computational graph? 

[source](https://blog.paperspace.com/pytorch-101-understanding-graphs-and-automatic-differentiation/)

## Tensors

- "On it's own, `Tensor` is just like a numpy `ndarray`. A data structure that can let you do fast linear algebra options. If you want PyTorch to create a graph corresponding to these operations, you will have to set the `requires_grad` attribute of the `Tensor` to True."
- "`requires_grad` is contagious. It means that when a `Tensor` is created by operating on other `Tensor`s, the `requires_grad` of the resultant `Tensor` would be set `True` given at least one of the tensors used for creation has it's `requires_grad` set to `True`."
- "Each `Tensor` has [...] an attribute called `grad_fn`, which refers to the mathematical operator that creates the variable [d.h. zB., wenn die Variable `d` über `d = w3*b + w4*c` definiert ist, dann ist das `grad_fn` von `d` der Additionsoperator `+`]. If `requires_grad` is set to False, `grad_fn` would be `None`." (kann man mit `print("The grad fn for a is", a.grad_fn)` testen!) (lies das nochmal genauer im Post!)
- "One can use the member function `is_leaf` to determine whether a variable is a leaf `Tensor` or not."

## `torch.nn.Autograd.Function` class

- "This class has two important member functions we need to look at.":
	- `forward`
		- "simply computes the output using it's inputs"
	- `backward`
		- "takes the incoming gradient coming from the the part of the network in front of it. As you can see, the gradient to be backpropagated from a function $f$ is basically the **gradient that is backpropagated to $f$ from the layers in front of it** multiplied by **the local gradient of the output of f with respect to it's inputs**. This is exactly what the `backward` function does." (lies das nochmal genauer nach!)
			- Let's again understand with our example of $$ d = f(w_3b , w_4c) $$
				1.  _d_ is our `Tensor` here. It's `grad_fn`  is `<ThAddBackward>`_._ This is basically the addition operation since the function that creates _d_ adds inputs.
				2.  The `forward` function of the it's `grad_fn`  receives the inputs $w_3b$ _and_ $w_4c$ and adds them. This value is basically stored in the _d_
				3.  The `backward` function of the `<ThAddBackward>`  basically takes the the **incoming gradient** from the further layers as the input. This is basically $\\frac{\\partial{L}}{\\partial{d}}$ coming along the edge leading from _L_ to _d._ This gradient is also the gradient of _L_ w.r.t to _d_ and is stored in `grad`  attribute of the `d`. It can be accessed by calling `d.grad`_._
				4.  It then takes computes the local gradients $\\frac{\\partial{d}}{\\partial{w\_4c}}$ and $\\frac{\\partial{d}}{\\partial{w\_3b}}$.
				5.  Then the backward function multiplies the incoming gradient with the **locally computed gradients** respectively and "_**sends**_" the gradients to it's inputs by invoking the backward method of the `grad_fn` of their inputs.
				6.  For example, the `backward` function of  `<ThAddBackward>`  associated with _d_ invokes backward function of the `grad_fn` of the $w\_4\*c$ (Here, $w\_4\*c$ is a intermediate Tensor, and it's `grad_fn` is `<ThMulBackward>`. At time of invocation of the `backward` function, the gradient $\\frac{\\partial{L}}{\\partial{d}} * \\frac{\\partial{d}}{\\partial{w_4c}} $ is passed as the input.
				7.  Now, for the variable $w\_4\*c$, $\\frac{\\partial{L}}{\\partial{d}} \* \\frac{\\partial{d}}{\\partial{w\_4c}} $ becomes the incoming gradient, like $\\frac{\\partial{L}}{\\partial{d}} $ was for $d$ in step 3 and the process repeats.

## How are PyTorch's graphs different from TensorFlow graphs

- PyTorch creates something called a **Dynamic Computation Graph**, which means that the graph is generated on the fly.
	- in contrast to the **Static Computation Graphs** used by TensorFlow where the graph is declared **before** running the program
- Until the `forward` function of a Variable is called, there exists no node for the `Tensor` (it’s `grad_fn`) in the graph.

        ```python
            a = torch.randn((3,3), requires_grad = True)   #No graph yet, as a is a leaf
            
            w1 = torch.randn((3,3), requires_grad = True)  #Same logic as above
            
            b = w1*a   #Graph with node `mulBackward` is created.
        ```    

- The graph is created as a result of `forward` function of many _Tensors_ being invoked. Only then, the buffers for the non-leaf nodes are allocated for the graph and intermediate values (used for computing gradients later). When you call `backward`, as the gradients are computed, these buffers (for non-leaf variables) are essentially freed, and the graph is _destroyed_ (In a sense, you can\'t backpropagate through it, since the buffers holding values to compute the gradients are gone).

- Next time, you will call `forward` on the same set of tensors, **the leaf node buffers from the previous run will be shared, while the non-leaf nodes buffers will be created again.**

- lies den Abschnitt im Post !

