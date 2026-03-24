function [X,U] = MPC_param(A,B,Ts,T,N,Q,R,u_min,u_max,x0,u_star)
% MPC_param: receding horizon con tracking opzionale via u_star.
% Restituisce X (nx x (T+1)) e U (nu x T).

% Dimensioni
nx = size(A,1);
nu = size(B,2);

% Peso terminale P
try
    if exist('idare','file')
        [P,~,~] = idare(A,B,Q,R,[],[]); % Riccati discreta (generalizzata)
    else
        [P,~,~] = dare(A,B,Q,R);
    end
catch
    warning('Impossibile calcolare P con idare/dare: uso P = Q come ripiego.');
    P = Q;
end

% Stato/ingresso di equilibrio associati a u_star
u_ss  = u_star * ones(nu,1);
IminusA = eye(nx) - A;
if rcond(IminusA) < 1e-12
    x_ss = pinv(IminusA) * (B*u_ss);
else
    x_ss = IminusA \ (B*u_ss);
end

% Variabili di errore (x_e = x - x_ss ; v = u - u_ss)
x_e = x0 - x_ss;

% Pesi aggregati con P terminale
Q_bar = blkdiag(kron(eye(N-1),Q), P);
R_bar = kron(eye(N), R);

% Bounds su sequenza di controllo v = u - u_ss
lb_u = kron(ones(N,1), u_min*ones(nu,1));
ub_u = kron(ones(N,1), u_max*ones(nu,1));
lb_v = lb_u - kron(ones(N,1), u_ss);
ub_v = ub_u - kron(ones(N,1), u_ss);

% Inizializzazioni
X = zeros(nx, T+1);  X(:,1) = x0;
U = zeros(nu, T);

options = optimoptions('quadprog','Display','off','ConstraintTolerance',1e-4);

% Ciclo receding horizon
for k = 1:T
    % Matrici di predizione
    Sx = zeros(N*nx, nx);
    Su = zeros(N*nx, N*nu);
    for i = 1:N
        Sx((i-1)*nx+1:i*nx,:) = A^i;
        for j = 1:i
            Su((i-1)*nx+1:i*nx,(j-1)*nu+1:j*nu) = A^(i-j)*B;
        end
    end

    % QP: H, f
    H = 2*(Su' * Q_bar * Su + R_bar);
    H = (H + H')/2;                 % simmetrizzazione numerica
    f = 2 * Su' * Q_bar * (Sx * x_e);

    % QP con soli bounds
    [v_opt,~,exitflag] = quadprog(H, f, [], [], [], [], lb_v, ub_v, [], options);
    if exitflag < 0
        warning('QP unfeasible/unbounded allo step %d (exitflag=%d). Stop.', k, exitflag);
        break;
   end

    % Receding horizon
    v_k = v_opt(1:nu);          % primo campione in v
    u_k = v_k + u_ss;           % ritorno a coordinate originali
    U(:,k) = u_k;

    % Evoluzione errore/stato
    x_e      = A*x_e + B*v_k;
    X(:,k+1) = x_ss + x_e;
end
end
