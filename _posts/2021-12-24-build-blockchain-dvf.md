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
  - Build
  - Blockchain
tags:
  - build
  - blockchain
toc: true
toc_label: "Contents"

---

# Postman Settings

To interact with the built `blockchain.py` API over a network via the Postman App:

- Select Authorization Type "No Auth" !
- select "raw" 
- select "JSON" instead of "Text" 

# Set up python environment

```bash
# create a python environment
python3 -m venv env

# activate the python environment
source env/bin/activate

# install necessary packages in this environment
# Note: try newer version of Flask (I used version 2.0.2), if something does not work in the following)
pip install Flask==0.12.2 requests==2.18.4 
```

# Modify old blockchain.py

- add the following `app.config` line:

```python
# Instantiate the Node
app = Flask(__name__)
app.config['JSONIFY_PRETTYPRINT_REGULAR'] = False
```

# Start the server

```bash
python3 blockchain.py -p 5001
```

# http requests 

## General structure

### Examples 

- GET `http://localhost:5000/chain`
- POST `http://localhost:5001/transactions/new`

## Examples

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
