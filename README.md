# Advanced Control Systems: MAS + MPC Project

Progetto di controllo avanzato sviluppato per il corso di **Advanced Control Systems** (a.a. 2024/2025) presso l'Università degli Studi **Roma Tre**.

**Candidato:** Marina Granozio  
**Data di consegna:** Febbraio 2026

---

## 🛰️ Sezione 1: Multi-Agent Systems (MAS)

L'obiettivo della prima parte è il coordinamento di un sistema multi-agente nel piano con dinamica del primo ordine:
$$\dot{x}_{i}=u_{i}$$

### Controllo a Potenziali
Il controllo garantisce la coesione dello sciame e l'evitamento delle collisioni tramite una legge a potenziali artificiali:
* **Aggregazione:** Garantita da un termine di attrazione costante $g_{A}=a$.
* **Separazione:** Gestita da un termine di repulsione esponenziale $g_{R}=b e^{-\frac{||y||^{2}}{c}}$.
* **Navigazione:** Una guida globale $u_{g}$ permette la traslazione dell'intero cluster senza deformarlo.

### Analisi di Dispersione
La compattezza dello sciame è valutata tramite metriche di dispersione ($R_{mean}, R_{RMS}, R_{max}$). Il raggio finale è limitato dal bound teorico:
$$R_{th}=\sqrt{\frac{b|E|}{a \lambda_{2}(L)}}$$
L'analisi mostra come una maggiore connettività algebrica ($\lambda_{2}$) e la rigidità del grafo ($|E| \ge 2N-3$) portino a un assestamento più rapido e a cluster più compatti.

---

## 🎮 Sezione 2: Model Predictive Control (MPC)

Sviluppo di un controllore predittivo per compiti di regolazione e tracking di un equilibrio $(x_{ss}, u^*)$ con vincoli sugli ingressi $u \in [-5, 5]$.

### Formulazione LQP
Il problema di ottimizzazione quadratica (QP) viene risolto a ogni passo secondo la filosofia del **Receding Horizon**:
$$\min_{U} \sum_{k=0}^{N-1} \left( x(k)^{\top}Q x(k) + u(k)^{\top}R u(k) \right) + x(N)^{\top}P x(N)$$

### Parametri Chiave
* **Orizzonte di predizione:** $N=12$.
* **Peso terminale $P$:** Calcolato tramite equazione discreta di Riccati (`idare`) per favorire la stabilità.
* **Traslazione delle coordinate:** Il tracking è gestito definendo l'errore $x_e = x - x_{ss}$.

---

## 📊 Analisi dei Risultati

### Stabilità Empirica
Il sistema è stato testato in tre configurazioni per valutare l'impatto dei pesi e dell'orizzonte:

| Caso | N | Q | R | Comportamento | Stabilità |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **1** | 10 | $5I$ | 0.01 | Ingressi saturati e divergenza | **Instabile** |
| **2** | 15 | $I$ | 0.5 | Lento con overshoot limitato | **Stabile Semplice** |
| **3** | 20 | $I$ | 0.1 | Convergenza rapida e precisa | **Asintotica** |

### Conclusioni su Collision Avoidance
I test sulla Collision Avoidance (CA) hanno mostrato che una repulsione "morbida" richiede un tuning accurato del guadagno $k_{obs}$ e del passo di campionamento $T$ per evitare violazioni della clearance in corridoi stretti.

---

## 🛠️ Requisiti
*
