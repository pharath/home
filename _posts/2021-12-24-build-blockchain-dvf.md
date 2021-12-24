---
title: "Learn Blockchains by Building One"
excerpt: "Original project by Daniel van Flymen: original [blog post](https://hackernoon.com/learn-blockchains-by-building-one-117428612f46)"
classes: wide
header:
  teaser: /assets/images/Blockchain.jpg
  overlay_image: /assets/images/Blockchain.jpg
  overlay_filter: 0.6
  caption: "Photo credit: [**muenchen.digital**](https://muenchen.digital/blog/explainit-blockchain-erklaert/)"
  actions:
    - label: "Download this project on Github"
      url: "https://github.com/pharath/blockchain"
categories:
  - Blockchain
tags:
  - blockchain

---

# Blockchain

- Postman: Authorization Type auf "No Auth" stellen !

- requests ("raw" auswählen und "Text" auf "JSON" stellen):

| command | description |
| :---: | :---: |
GET /chain | zeigt die gesamte Blockchain
GET /mine | erzeugt einen neuen Block
POST /transactions/new | fügt eine Transaktion zum nächsten Block (der noch nicht in der `GET /chain` Liste ist) hinzu, der geminet wird. Der nächste `GET /mine` Call erzeugt dann den Block, in dem diese transaction ist. Beispieltransaktion:

```yaml
{
    "sender": "d4ee26eee15148ee92c6cd394edd974e",
    "recipient": "someone-other-address",
    "amount": 5
}
``` 

Spin up another node on machine, on a different port [port 5001], and register it with current node [on port 5000]:

| command | description |
| :---: | :---: |
POST nodes/register |

```yaml
{
    "nodes": ["http://127.0.0.1:5001/"]
}
```

oder alternativ:

```yaml
{
    "nodes": ["http://192.168.2.126:5001/"]
}
```

| command | description |
| :---: | :---: |
GET /nodes/resolve | replace shorter chains with the longest chain by the Consensus Algorithm
