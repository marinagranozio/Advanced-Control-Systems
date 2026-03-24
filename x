function [G_fin] = MAS_Graph(N, P)

    % Genera grafo non diretto, simmetrico e connesso
    conn = false;

        while ~conn
            A = rand(N)<P; % generazione matrice dinamica
            D = triu(A); % matrice triangolare
            E = D'; % trasp matrice triangolare
            F = D+D'; % costruzione matrice simmetrica
            G = diag(diag(F)); % generazione matrice diagonale
            G_fin =  F - G; % matrice di adiacenza
            Grafo = graph(G_fin); % generazione grafo a partire da matrice di adiacenza
            L = Grafo.laplacian; % laplaciano
            eigL = eig(L); % vettore autovalori laplaciano
            conn = eigL(2) > 0; % unica componente connessa
               
            %   % Visualizzazione autovalori
            % fprintf('lambda_1 = %.5f\n', eigL(1));
            % fprintf('lambda_2 = %.5f\n', eigL(2));
        end
end
