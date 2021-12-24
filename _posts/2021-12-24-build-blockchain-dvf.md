---
title: "Learn Blockchains by Building One"
excerpt: "By Daniel van Flymen: original [blog post](https://hackernoon.com/learn-blockchains-by-building-one-117428612f46)"
classes: wide
header:
  teaser: /assets/images/Blockchain.jpg
  overlay_image: /assets/images/Blockchain.jpg
  overlay_filter: 0.6
  actions:
    - label: "Download this project on Github"
      url: "https://github.com/pharath/blockchain"
categories:
  - Blockchain
tags:
  - blockchain

---

Blockchain:

-Postman: Authorization Type auf "No Auth" stellen !

-requests ("raw" auswählen und "Text" auf "JSON" stellen):

GET /chain:
zeigt die gesamte Blockchain

GET /mine:
erzeugt einen neuen Block

POST /transactions/new:
{
    "sender": "d4ee26eee15148ee92c6cd394edd974e",
    "recipient": "someone-other-address",
    "amount": 5
}
fügt eine Transaktion zum nächsten Block (der noch nicht in der GET /chain Liste ist) hinzu, der geminet wird. Der nächste GET /mine Call erzeugt dann den Block, in dem diese transaction ist.

Spin up another node on machine, on a different port [port 5001], and register it with current node [on port 5000]:
POST nodes/register:

{
    "nodes": ["http://127.0.0.1:5001/"]
}

oder alternativ:

{
    "nodes": ["http://192.168.2.126:5001/"]
}

GET /nodes/resolve:
replace shorter chains with the longest chain by the Consensus Algorithm
