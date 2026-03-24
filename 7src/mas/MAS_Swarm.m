function [x, t, dispersione] = MAS_Swarm(G_adj, x0, T, tfin, ug, param, obs)
% Aggregazione swarm con potenziale 

t = 0:T:tfin;
N = size(G_adj, 1);
x = zeros(N, 2, length(t));
x(:,:,1) = x0;

% Estrazione parametri potenziale 
a = param.a; % Coeff. Attrazione
b = param.b; % Coeff. Repulsione
c = param.c; % Coeff. decadimento repulsione

% Parametri Ostacoli
obs_range = param.obs_range;
k_obs = param.k_obs; 

% Dispersione swarm
dispersione = zeros(1, length(t));

for k = 2:length(t)
    % Calcolo baricentro attuale per metriche
    centroid = mean(x(:,:,k-1), 1);
    dispersione(k) = mean(vecnorm(x(:,:,k-1) - centroid, 2, 2));

    for i = 1:N
        u_agg = [0 0]; % Controllo aggregazione locale
        
        % --- 1. AGGREGAZIONE (Attrazione - Repulsione) ---
        % Loop sui vicini definiti dal grafo
        neighbors = find(G_adj(i,:)); 
        for j = neighbors
            y = x(j,:,k-1) - x(i,:,k-1); % Vettore distanza y = xj - xi
            dist = norm(y); % Distanza scalare 
            
            if dist > 0
                % Funzioni scalari g_A e g_R
                g_A = a; 
                g_R = b * exp(-(dist^2)/c);
                
                % Legge di controllo: u = sum (g_A - g_R) * (xj - xi)
                u_agg = u_agg + (g_A - g_R) * y;
            end
        end

        % --- 2. COLLISION AVOIDANCE OSTACOLI (Virtual Agents) ---
        u_obs = [0 0];
        if ~isempty(obs) % Il codice si attiva solo se la lista degli ostacoli non è vuota
            for o = 1:size(obs, 1)
                delta_o = x(i,:,k-1) - obs(o,:); % Vettore che va dall'ostacolo all'agente
                dist_o = norm(delta_o);
                if dist_o < obs_range && dist_o > 0   % Sensing range
                    % Solo repulsione pura dagli ostacoli
                    u_obs = u_obs + k_obs * (delta_o / dist_o^2); 
                end
            end
        end
                
        % u_i = Aggregazione + Guida + Ostacoli
        u_total = u_agg + ug + u_obs;
        
        % Integrazione Eulero
        x(i,:,k) = x(i,:,k-1) + u_total * T;
    end
end
end