# Advanced Control Systems: MAS + MPC Project

[cite_start]Questo repository contiene il materiale del progetto sviluppato per il corso di **Advanced Control Systems** (a.a. 2024/2025) presso l'Università degli Studi **Roma Tre**[cite: 1, 3, 4].

[cite_start]**Studente:** Marina Granozio [cite: 2]  
[cite_start]**Data:** Febbraio 2026 [cite: 15]

---

## 🛰️ Parte 1: Multi-Agent Systems (MAS)

[cite_start]L'obiettivo è il controllo di uno sciame di $N=10$ agenti nel piano con dinamica del primo ordine $\dot{x}_{i}=u_{i}$[cite: 7, 8, 33].

### Modello e Controllo a Potenziali
[cite_start]Il controllo locale si basa su funzioni di potenziale per garantire coesione e separazione[cite: 9, 61]:
$$u_{i}=\sum_{j\in N_{i}}g(||y_{ij}||)y_{ij}, \quad y_{ij}=x_{j}-x_{i}$$
[cite_start]Dove la funzione di guadagno $g(||y||)$ è composta da[cite: 62]:
* [cite_start]**Attrazione:** $g_{A}=a$ (costante)[cite: 62].
* [cite_start]**Repulsione:** $g_{R}=b e^{-\frac{||y||^{2}}{c}}$[cite: 62].

[cite_start]La guida globale per la traslazione del gruppo è aggiunta come $u_{i}=u_{agg,i}+u_{g}$[cite: 63, 64].

### Analisi di Aggregazione
[cite_start]Il sistema converge in una "bolla" attorno al centroide $\overline{x}$ con un bound teorico conservativo[cite: 73, 74]:
$$||x-\overline{x}||\le R_{th}=\sqrt{\frac{b|E|}{a \lambda_{2}(L)}}$$
* [cite_start]**$|E|$**: Numero di archi (tende ad espandere il cluster)[cite: 75].
* [cite_start]**$\lambda_{2}$**: Connettività algebrica (tende a compattare il cluster)[cite: 75].

---

## 🎮 Parte 2: Model Predictive Control (MPC)

[cite_start]Implementazione di un controllo predittivo per compiti di **Regolazione** (verso lo zero) e **Tracking** (verso un equilibrio $x_{ss}$)[cite: 380, 381].

### Problema Lineare Quadratico (LQP)
[cite_start]Il problema di ottimizzazione a tempo discreto con vincoli sull'ingresso $u \in [-5, 5]$ è definito come[cite: 379, 384]:
$$\min_{u} \sum_{k=0}^{N-1} \left( x(k)^{\top}Q x(k) + u(k)^{\top}R u(k) \right) + x(N)^{\top}P x(N)$$
$$\text{s.t. } \begin{cases} x(k+1) = Ax(k) + Bu(k) \\ u_{min} \le u(k) \le u_{max} \end{cases}$$

### [cite_start]Parametri di Simulazione [cite: 402, 407, 409]
* **Orizzonte di predizione ($N$):** 12 passi.
* **Tempo di campionamento ($T_s$):** 0.1s.
* **Peso terminale ($P$):** Calcolato tramite l'equazione di Riccati (`idare`).

---

## 📊 Risultati e Analisi Parametrica

### Confronto Stabilità MPC
[cite_start]Il progetto analizza come la scelta dell'orizzonte e dei pesi influenzi la stabilità del sistema[cite: 706]:

| Caso | $N$ | $Q$ | $R$ | Esito Empirico | [cite_start]Stabilità [cite: 695, 696, 697] |
| :--- | :--- | :--- | :--- | :--- | :--- |
| 1 | 10 | $5I$ | 0.01 | Divergenza rapida | **Instabile** |
| 2 | 15 | $I$ | 0.5 | Lento ma limitato | **Semplice (Locale)** |
| 3 | 20 | $I$ | 0.1 | Convergenza ottimale | **Asintotica** |

### Collision Avoidance (CA)
[cite_start]L'analisi ha evidenziato che la CA a potenziale "morbido" può fallire se il guadagno $k_{obs}$ è insufficiente rispetto alla velocità di guida $u_g$ o se il corridoio tra gli ostacoli è troppo stretto[cite: 328, 329, 330].

---

## 🛠️ Setup del Progetto
1. [cite_start]Eseguire lo script di inizializzazione per generare il grafo di comunicazione e le posizioni iniziali $x_0 \in [-2.5, 2.5]^2$[cite: 56].
2. [cite_start]Lanciare le simulazioni MAS per verificare la coesione del gruppo.
3. [cite_start]Utilizzare i solver QP (come `quadprog` in MATLAB) per eseguire i test di regolazione e tracking MPC[cite: 382].

```matlab
% Esempio di calcolo della matrice Hessiana per MPC nelle slide
H = 2 * (Su' * Q_bar * Su + R_bar); % [cite: 431]
f = 2 * Su' * Q_bar * Sx * x_ek;    % [cite: 431]
