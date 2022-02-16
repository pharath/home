---
title: "Machine Learning (Part 3)"
excerpt: "Notes on Computer Vision theory and NLP. Based on Goodfellow, Bengio \"Deep Learning\", Stanford CS231n and RWTH Aachen University Machine Learning"
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
  - Computer_Vision
tags:
  - lecture_notes
  - ml
  - cv
toc: true
toc_label: "Contents"

---

# CNN

![conv_layer_params_cs231n_karpathy.png](/assets/images/conv_layer_params_cs231n_karpathy.png)

## Why is the size of $F$ always an odd number?

- usually $1$, $3$, $5$, $7$ and sometimes $11$
- you can use filters with even numbers, but it is not common
- the lowest people use is $3$, mainly for convenience
    - you want to apply the filter around a well-defined position that "exists" in the input volume (see traditional image processing)

## Why pad with zeros ?

- you do not want the outer padded pixels to contribute to the dot product
- but you can also use other padding techniques, but it is less common, in practice

## Why ConvNets work?

[[source](https://ranzato.github.io/publications/ranzato_deeplearn17_lec1_vision.pdf)]

![whyconvnetswork](/home/assets/images/CV/whyconvnetswork.png)

## Data Augmentation

- cropping
- zooming
- flipping
- Color PCA
    - for each pixel an Eigenspace is computed that highlights the expected color variations and then within those color variations you can adjust the colors of the entire image

### Test-time augmentation (TTA)

- apply **cropping and flipping** also at test time
- **ColorPCA** at test time can improve accuracy another $1\%$, but runtime will increase!
- Open question: Is **zooming** at test time not necessary because CNNs are invariant to translations?

## Mask R-CNN

- used for
    - Object Detection
        - by using Faster R-CNN
    - Instance Segmentation
        - by using an **additional head** that predicts a segmentation mask for each region proposal
    - Pose Estimation
        - by using an **additional head** that predicts the coordinates of the joints of the instance

## ResNet

### He et al

- Plain nets: stacking $3\times 3$ Conv layers
    - 56-layer net has higher **training** error than 20-layer net
        - not caused by overfitting
            - when you are overfitting you would expect to have a very low training error rate and just bad test error
        - unlikely to be caused by vanishing gradients
            - plain nets are trained with BN, which ensures that forward propageted signals have non-zero variances
                - i.e. forward signals do not vanish!
            - He et al verify that the gradients exhibit healthy norms
                - i.e. backward signals do not vanish!
        - the solver works to some extent
            - "In fact, the 34-layer plain net is still able to achieve competitive accuracy (Table 3), suggesting that **the solver works** to some extent."
        - deeper model has a richer solution space which should allow it to find better solutions (i.e. it should perform at least as well as a shallower model)
            - **solution by construction**:
                - copy the original layers from the shallower model and just set the extra layers as identity mappings
                    - this model should be at least as well as the shallower model
                - this motivates the ResNet design: 
                    - fit a residual mapping instead of a direct mapping, so that it is **easier for the deep NN to learn an identity mapping on unused layers** ("learn some Delta/residual" instead of "learn an arbitrary mapping")
                    - so we can get something close to this "**solution by construction**" that we had earlier
        - **reason**: optimization difficulties
            - "deep plain nets may have **exponentially low convergence rates**"
                - d.h. der Solver funktioniert, aber er braucht einfach nur so lange, dass es so aussieht als würde er nicht konvergieren

### Veit et al

- "depth is good" (cf. He et al) is **not the full explanation** 
    - because, if this were true, 1202 layers should have been better than 110 layers
        - however, accuracy stagnates above about 150 layers
- [Veit et al, 2016](https://arxiv.org/pdf/1605.06431.pdf) did some experiments on ImageNet and CIFAR-10
- Unraveling ResNets:
    - ResNets **can be viewed as a collection of shorter paths through different subsets** of the layers
    - see "unraveled view" of a ResNet in slides
- Deleting layers:
    - VGG-Net:
        - when deleting a layer in **VGG-Net**, it breaks down completely
    - ResNets:
        - in **ResNets** deleting has almost no effect (except for pooling layers)
            - "something important" seems to happen in pooling layers
            - see "pooling layer spike" in slides
        - deleting an increasing number of layers increases the error smoothly
        - i.e. **paths in a ResNet do not strongly depend on each other**
- Gradient contribution of each path:
    - **only a small portion ($\lt 1\%$) of all paths** in the NN are used for passing the gradients
        - effectively **only shallow paths** with 5-17 modules are used
            - i.e. it remains a hard problem to pass gradients through weight layers and apparently the optimum is in the range 5-17 weight layers
        - this **explains why deleting layers has almost no effect**
            - deleting only affects a subset of paths and the shorter paths are less likely to be affected than the longer paths
    - distribution of path lengths follows a Binomial distribution
        - where $\text{path length} = \text{\#(weight layers along the path)}$
- new interpretation of ResNet:
    - **ResNet can be viewed as an ensemble method**
        - they create an ensemble of relatively **shallow paths**
        - making ResNet deeper increases the ensemble size
            - Recall ensemble learning: this **explains why the accuracy stagnates** above about 150 layers or so 
                - there is only so much more that a new model (here: path) can add to the ensemble model accuracy
        - similarly **Dropout** can be viewed as an ensemble method
            - however, deleting connections using **Dropout can improve the performance of ResNet**
    - excluding longer paths does not negatively affect the results

## Transfer learning

- This is often done in medical imaging and **in situations where there is very little training data available**.

1. pretrain the network
2. fine-tuning:
    - little data available:
        - swap only the Softmax layer at the end and leave everything else unchanged
        - fine-tune this final Softmax layer based on the new data
    - medium sized data set available:
        - retrain a bigger portion of the network
        - retrain the full network, but use the old weights as initialization
            - restrict the step size, so that you allow only small changes in the early layers

## FCN

- used for
    - Semantic Segmentation
- produce a **heatmap** of class labels (i.e. a $(C\times H\times W)$ tensor, where $C$ is the number of classes and $H\times W$ are the input image dimensions) for each pixel in the input image
- [source](https://ai.stackexchange.com/questions/21810/what-is-a-fully-convolution-network)
    - A **fully convolutional network (FCN)** is a neural network that only performs convolution (and subsampling or upsampling) operations. Equivalently, an FCN is a CNN without fully connected layers.

### FCN vs CNN

- [Andrew Ng lecture](https://www.youtube.com/watch?v=XdsmlBGOK-k):
    - FCNs can be viewed as performing a sliding-window classification and produce a heatmap of output scores for each class
    - "**convolutionalization** of FC layers makes FCNs **more efficient than standard CNNs**
        - computations are reused between the sliding windows
- [source](https://ai.stackexchange.com/questions/21810/what-is-a-fully-convolution-network)
    - The typical **convolutional neural network (CNN)** is **not fully convolutional** because it often contains **fully connected layers** too (which do not perform the convolution operation), which are **parameter-rich**, in the sense that they have many parameters (compared to their equivalent convolution layers), 
        - although the fully connected layers [can also be viewed as convolutions with kernels that cover the entire input regions](https://arxiv.org/pdf/1411.4038.pdf), which is the main idea behind converting a CNN to an FCN. 
            - See [this video](https://www.youtube.com/watch?v=XdsmlBGOK-k) by Andrew Ng that explains how to convert a fully connected layer to a convolutional layer.

## Encoder-Decoder Architecture

- used for
    - Semantic Segmentation
- **problem**: FCN output has low resolution
- **solution**: perform upsampling to get back to the desired resolution
    - use skip connections to **preserve higher resolution information**

## N-gram model

- problem 1: **scalability**: 
    - possible combinations and thus also the required data increases exponentially
- problem 2: **partial observability**:
    - probability is not zero just because the count is zero
        - Solution 1: need to back-off to $N-1$-grams when the count for $N$-gram is too small
        - Solution 2: use Kneser-Ney smoothing (compensate for uneven sampling frequencies)

## Word Embeddings

- watch:
    - [Word Representation](https://www.youtube.com/watch?v=jQTuRnjJzBU)
    - [Learning word embeddings](https://www.youtube.com/watch?v=yXV_Torwzyc)
- Neural Probabilistic Language Model (Bengio 2003)
    - for each $\mathbf{x}$ only one row of $\mathbf{W}_{V\times d}$ is needed
        - $\mathbf{W}_{V\times d}$ is effectively a **look-up table**
        - $V\approx 1\text{M}$ (size of the vocabulary, i.e. $\mathbf{x}$ is a one-hot encoded 1M-dimensional vector!)
        - $d\in (50,300)$ (feature vector size)
- Analogy questions:
    - Vektordifferenz $\mathbf{a}-\mathbf{b}$ geht "zu $\mathbf{a}$ hin"
    - search the embedding vector space for the word closest to the result **using the cosine distance**
    - types of analogies:
        - semantic (use **skip-gram**):
            - Athens - Greece $\approx$ Oslo - Norway
            - brother - sister $\approx$ grandson - granddaughter
        - syntactic (use **CBOW**):
            - great - greater $\approx$ tough - tougher
            - Switzerland - Swiss $\approx$ Cambodia - Cambodian
- **problem**: calculating the denominator of the softmax (i.e. normalization over 100k-1M outputs, sum over the entire vocab size in the denominator) is very expensive
    - **Solution 1**: use **Hierarchical Softmax** [[paper](https://arxiv.org/pdf/1411.2738.pdf)] instead of standard Softmax:
        - watch: [Andrew Ng](https://www.youtube.com/watch?v=3eoX_waysy4&list=PLhWB2ZsrULv-wEM8JDKA1zk8_2Lc88I-s&index=6)
        - learn a binary search tree
            - [each leaf is one word]
                - [path to the leaf determines the **probability of the word being the output word**]
        - factorize "probability of a word being the output word" as a **product of node probabilities along the path**
        - learn a **linear decision function at each node** for deciding for left or right child node
        - computational cost: $\mathcal{O}(\log(V))$ (instead of $\mathcal{O}(V)$ for standard Softmax)
    - **Solution 2**: use **negative sampling**

## Embeddings in Vision

- **Siamese networks**
    - similar idea to word embeddings
        - learn an embedding network that preserves similarity between inputs
    - Learning
        1. with triplet loss
            - learn an embedding that groups the **positive** closer to the **anchor** than the **negative**
            - $d(a,p)+\alpha\lt d(a,n)$ $\Rightarrow$ $L_{tri}=\sum_{a,p,n}\max(\alpha+D_{a,p}-D_{a,n},0)$
                - $\alpha$: margin 
                    - to avoid trivial solution of the inequality (e.g. same output for all images)
                    - $\alpha$ determines how different the inputs have to be in order to satisfy the inequality (similar to SVM margin)
            - **problem**: most triplets are uninformative
                - we want **medium-hard** triplets because using too hard triplets is like focussing on outliers (which do not help learning)
                - **solution**: use hard triplet mining
                    - process dataset to find hard triplets
                    - use those for learning
                    - iterate
            - used in e.g. Google FaceNet
        2. with contrastive loss
    - apps:
        - patch matching
        - face recognition

# RNN

## Architecture

![karpathy_rnn.png](/home/assets/images/karpathy_rnn.png)<a name="karpathy_rnn"></a>

- $\mathbf{h}_0$ is also **learned** like the other parameters
- weights $\mathbf{W}_{hh}$ between the hidden units are **shared** between temporal layers
- connection matrices: $\mathbf{W}_{xh}$, $\mathbf{W}_{hy}$ and $\mathbf{W}_{hh}$
- powerful because
    - distributed hidden state 
        - allows to store information about the past efficiently
    - nonlinearities
        - hidden states can be updated in complicated ways
    - RNNs are Turing complete
        - given enough neurons and time RNNs can compute anything that can be computed by a computer
- Hinton: 
    - just a feedforward net that keeps reusing the same weights in each (temporal) layer

## BPTT

- BPTT equations:
    - **key point**: 
        - in $\frac{\partial E}{\partial w_{ij}}$ the **(temporal) error propagation term** $\frac{\partial h_t}{\partial h_k}$ will either go to zero or explode (depending on the largest singular value of $\mathbf{W}_{rec}$) for $k\ll t$ (i.e. for **long term contributions**)
            - vanishing gradient proof (cf. below in Pascanu paper): $\frac{\partial h_t}{\partial h_k}$ is bounded from above by $\eta^{t-k}$ which goes to 0 exponentially fast with $t-k$ and, therefore, long term contributions go to zero, too
    - Bengio_1994 and [Pascanu_2013](https://proceedings.mlr.press/v28/pascanu13.pdf):  
    ![bengio94_2.png](/home/assets/images/bengio94_2.png)
    ![bengio94_3.png](/home/assets/images/bengio94_3.png)
    ![bengio94_1.png](/home/assets/images/bengio94_1.png)
    ![bengio94_4.png](/home/assets/images/bengio94_4.png)
    ![bengio94_5.png](/home/assets/images/bengio94_5.png)
    - i.e. this is only valid for **long term contributions** ($k\ll t$)

## Cliffs and Exploding Gradients

- if the largest Eigenvalue of the weight matrix $W_{hh}$ is $\gt 1$, the gradient will explode (s. [Karpathy code](#karpathy_rnn))
- source: [Goodfellow_2016](#Goodfellow_2016)

![RNN_cliff.png](/home/assets/images/goodfellow_ml/RNN_cliff.png)

![RNN_cliff_gradient_clipping.png](/home/assets/images/goodfellow_ml/RNN_cliff_gradient_clipping.png)

- [Goodfellow_2016](#Goodfellow_2016):
    - cliff structures are an **example of the <mark>exploding</mark> gradient phenomenon**
    - **Recurrent networks** use the same matrix $W$ at each time step, but **feedforward networks** do not, 
        - so even very deep feedforward networks can largely avoid the vanishing and exploding gradient problem

### Solution: Gradient Clipping

- rescale gradients to a fixed size
    - "if the gradient is larger than a threshold, clip it to that threshold"

### ReLUs

- [Le, Hinton 2015](https://arxiv.org/pdf/1504.00941.pdf):
    - **At first sight, ReLUs seem inappropriate for RNNs** because they can have very large outputs so they might be expected to be **far more likely to explode** than units that have bounded values

## Vanishing Gradient Problem

- if the largest Eigenvalue of the weight matrix $W_{hh}$ is $\lt 1$, the gradient will vanish (s. [Karpathy code](#karpathy_rnn))
- Goodfellow:
    - gradients through such a [RNN] graph are also scaled according to $\text{diag}(\vec{\lambda})^t$ 
    - **Vanishing gradients** make it difficult to know which direction the parameters should move to improve the cost function, while **exploding gradients** can make learning unstable. 
- Leibe:
    - they severely restrict the dependencies the RNN can learn
        - e.g. in language models:
            - words from time steps far away (**long-range dependencies**) are not taken into consideration when training to predict the next word
    - problem gets more severe the deeper the network is
    - harder problem than exploding gradients because it can be very hard to diagnose that vanishing gradients occur
        - you just see that learning gets stuck

## Long-range Dependencies

- [Le, Hinton 2015](https://arxiv.org/pdf/1504.00941.pdf):
    - [**key point**:]
        - [use ReLU and initialize $\mathbf{W}_{hh}$ to be the identity matrix (and biases to be zero)]
            - [this way RNNs can perform as well as LSTMs and learn long-range dependencies]
            - [Leibe: to propagate the gradients with a constant factor]
    - In this paper, we demonstrate that, with the right initialization of the weights, **<mark>RNNs</mark>** composed of rectified linear units are relatively easy to train and are **<mark>good at modeling long-range dependencies</mark>**.
    - Their **<mark>performance</mark>** on test data is **<mark>comparable with LSTMs</mark>**, both for toy problems involving very long-range temporal structures and for real tasks like predicting the next word in a very large corpus of text.
    - [method:]
        - We **initialize** the recurrent weight matrix to be the **identity matrix** and biases to be zero. 
            - This means that each new hidden state vector is obtained by simply copying the previous hidden vector then adding on the effect of the current inputs and replacing all negative states by zero. 
        - **In the absence of input**, an RNN that is composed of ReLUs and initialized with the identity matrix (which we call an **IRNN**) just **stays in the same state indefinitely**. 
        - The identity initialization has the very desirable property that when the **error derivatives for the hidden units** are backpropagated through time they **remain constant** provided no extra error-derivatives are added. 
            - This is the **same behavior as LSTMs** when their forget gates are set so that there is no decay and it **makes it easy to learn very long-range temporal dependencies**.
 
## Apps

- Predicting the next word (e.g. Google search suggestions)
- Machine translation (Sutskever 2014)
- Character-Level Language Model 
    - **task**: model the probability distribution of the next character in the sequence
        - advantage RNN: RNN can learn varying amount of context
    - Karpathy: 
        - min-char-rnn, 
        - generating Shakespearesque texts (3-layer RNN), 
        - generate Wikipedia text (LSTM), 
        - generate algebraic geometry text/LaTeX code (multilayer LSTM), 
        - generate Linux source code/C code (3-layer LSTM)
- image captioning
    - use CNN to define $h_0$
    - use RNN to produce a caption
    - data: e.g. Microsoft COCO
    - variant: "image to story"
- video to text description

## Units with a Gating Mechanism

- basic idea:
    - use more sophisticated units that implement a gating mechanism, such as a **long short-term memory (LSTM) unit** and a recently proposed **gated recurrent unit (GRU)** instead of more traditional recurrent units such as **tanh units**.
    - Johnson: [LSTMs are] designed to help alleviate this problem of vanishing and exploding gradients [of RNNs]

### LSTM

- [Olah](https://colah.github.io/posts/2015-08-Understanding-LSTMs/):
    - LSTMs are explicitly designed to avoid the long-term dependency problem. Remembering information for long periods of time is practically their default behavior, not something they struggle to learn!
        - [Le, Hinton](https://arxiv.org/pdf/1504.00941.pdf):
            - at the time [LSTMs were invented], the important issue was to find any scheme that could learn long-range dependencies rather than to find the minimal or optimal scheme.
- Leibe: 
    - with standard RNNs you can learn temporal dependencies of up to 10 time steps, with LSTMs you can maintain temporal connections of up to a 100 time steps
- **four** gates: i, f, o and g
- [Olah](https://colah.github.io/posts/2015-08-Understanding-LSTMs/):
    - Consider e.g. a **language model** trying to predict the next word based on all the previous ones
    - **f**: decide what information we’re going to throw away from the cell state
        - In such a [language model] problem, the cell state might include the gender of the present subject, so that the correct pronouns can be used. When we see a new subject, we want to forget the gender of the old subject.
    - **i, g**: decide what new information we’re going to store in the cell state
        - we’d want to add the gender of the new subject [$x_t$] to the cell state, to replace the old one we’re forgetting
    - **update** the old cell state, $C_{t−1}$, into the new cell state $C_t$
        - this is where we’d actually drop the information about the old subject’s gender and add the new information, as we decided in the previous steps [**steps f and i, g**]
    - **o**: decide what we’re going to output
        - since it just saw a subject, it might want to output information relevant to a verb, in case that’s what is coming next. For example, it might output whether the subject is singular or plural, so that we know what form a verb should be conjugated into if that’s what follows next.

#### Optimization

- [Le, Hinton](https://arxiv.org/pdf/1504.00941.pdf):
    - it is observed that setting a higher initial forget gate bias for LSTMs can give better results for long term dependency problems. We therefore also performed a grid search for the initial forget gate bias in LSTMs from the set {1.0, 4.0, 10.0, 20.0}

### GRU

- [Olah](https://colah.github.io/posts/2015-08-Understanding-LSTMs/):
    - Simpler model than LSTM
        - $z_t$: combine forget and input gate into a single **update gate** $z_t$
        - $r_t$: reset gate $r_t$ (functional form like update gate $z_t$)
        - merges cell state $C_{t-1}$ and hidden state $h_{t-1}$
- Empirical results
    - performance similar to LSTM
    - **But**: fewer parameters than LSTM (need less training data)
        - however, GRUs have more complex mechanisms that are harder to learn than LSTMs
            - therefore, performance similar to LSTMs
- Effects (see equations for $\tilde{h}_t$ and $h_t$)
    - reset: if $r_t$ is close to zero, ignore previous hidden state $h_{t-1}$
    - update: if $z_t$ is close to zero, $h_{t}\approx h_{t-1}$ 
        - i.e. update gate $z_t$ controls how much of the past state $h_{t-1}$ should matter now
        - in this way information can be copied through many time steps
            - i.e. less vanishing gradients
- typical learned behaviors (cf. "**GRU circuit**" visualization)
    - units with **short-term** dependencies often have an active reset gate $r_t=1$
    - units with **long-term** dependencies often have an inactive update gate $z_t=0$
