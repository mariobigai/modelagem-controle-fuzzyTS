import control as ctr
from modelos import auxiliar
import numpy as np
import scipy as sp


## Função para somar matrizes dinâmicamente no modelo TS
## Provavelmente tenha isso otimizado no Numpy, verificar e substituir depois
def somar(A, B):
    C = []
    # verificar se A e B tem a mesma ordem
    nLinhasA, nLinhasB = len(A), len(B)
    nColA, nColB = len(A[0]), len(B[0])
    if (nLinhasA == nLinhasB) and (nColA == nColB):
        # posso somar
        for i in range(nLinhasA):
            linha = [0] * nColA
            C.append(linha)
            for j in range(nColA):
                C[i][j] = A[i][j] + B[i][j]
    else:
        print("Matrizes não tem a mesma ordem")

    return C

class Buck_Boost_ideal:
    def __init__(self, R, C, L, Vin):
        self._R = R
        self._C = C
        self._L = L
        self._Vin = Vin

    def forced_response_TS(self, d_t, T=None, U=0., X0=0., transpose=False, interpolate=False, return_x=None, squeeze=None):

        """
        Método para gerar a resposta em malha aberta do modelo TS do Buck-Boost operando em várias regiões de operação

        :param D_t: Regiões de opeação (Duty-cycle)
        :param T: Tempo de simulação
        :param U: Entrada do sistema
        :param X0: (Biblioteca control)
        :param transpose: (Biblioteca control)
        :param interpolate: (Biblioteca control)
        :param return_x: (Biblioteca control)
        :param squeeze: (Biblioteca control)
        :return: Resultado da simulação: x, y
        """
        # Região mínima de operação
        Il_min = (self._Vin / self._R) * (d_t[0] / (1 - d_t[0]))
        Vc_min = self._Vin * (d_t[0] / (1-d_t[0]))

        # Região máxima de operação
        Il_max = (self._Vin / self._R) * (d_t[-1] / (1 - d_t[-1]))
        Vc_max = self._Vin * (d_t[-1] / (1-d_t[-1]))

        # Máximos e mínimos de g11
        b111 = (self._Vin + Vc_max) / self._L
        b112 = (self._Vin + Vc_min) / self._L

        # Máximos e mínimos de g12
        b211 = (-1 * Il_min) / self._C
        b212 = (-1 * Il_max) / self._C

        # Modelos lineares locais
        B1 = np.asarray([[b111], [b211]])
        B2 = np.asarray([[b112], [b211]])
        B3 = np.asarray([[b111], [b212]])
        B4 = np.asarray([[b112], [b212]])

        # Considerando o inicialmente aplha1 = 1, alpha2,3,4 =0
        A_TS = [[0, -1/self._L],
                [1/self._C, -1/(self._R*self._C)]]
        B_TS = somar(0 * B4, somar(0 * B3, somar(1 * B1, 0 * B2)))
        C_TS = [[0,1],[1,0]]
        E_TS = [[0],[0]]
        SS_TS = ctr.StateSpace(A_TS, B_TS, C_TS, E_TS)

        # Pega as matrizes de estado
        A, B, C, E = np.asarray(SS_TS.A), np.asarray(SS_TS.B), np.asarray(SS_TS.C), np.asarray(SS_TS.D)

        ## ------------------------------------------------------------------------------------------------------------
        # Verificações

        #Numero de estados
        n_states = A.shape[0]

        #Numero de Entradas
        n_inputs = B.shape[1]

        #Numero de Saídas
        n_outputs = C.shape[0]

        # Converte as entradas e tempo de simulação em numpy arrays
        if U is not None:
            U = np.asarray(U) # cria array de entrada
        if T is not None:
            T = np.asarray(T) # cria array de tempo

        #Garante que U e T tem o mesmo tamanho
        if (U.ndim == 1 and U.shape[0] != T.shape[0]) or \
                (U.ndim > 1 and U.shape[1] != T.shape[0]):
            ValueError('Pamameter ``T`` must have same elements as'
                       ' the number of columns in input array ``U``')

        # Erro se T é nulo
        if T is None:
            raise ValueError('Parameter ``T``: must be array-like, and contain '
                             '(strictly monotonic) increasing numbers.')

        # Verifica e atribui o T no formato correto
        T = auxiliar._check_convert_array(T, [('any',), (1, 'any')], 'Parameter ``T``: ', squeeze=True, transpose=transpose)

        dt = T[1] - T[0] #Passo de simulação

        # Erro se T não for igualmente espaçado - Fixed-step size
        if not np.allclose(T[1:] - T[:-1], dt):
            raise ValueError("Parameter ``T``: time values must be "
                             "equally spaced.")

        n_steps = T.shape[0]  # número de passos de simulação

        # Cria vetor X0 se não for dado e testa se está no formato correto
        X0 = auxiliar._check_convert_array(X0, [(n_states,), (n_states, 1)], 'Parameter ``X0``: ', squeeze=True)

        xout = np.zeros((n_states, n_steps))
        xout[:, 0] = X0
        yout = np.zeros((n_outputs, n_steps))

        ## Considerando apenas o caso contínuo com U diferente de zero
        legal_shapes = [(n_steps,), (1, n_steps)] if n_inputs == 1 else \
                       [(n_inputs, n_steps)]

        U = auxiliar._check_convert_array(U, legal_shapes,
                                 'Parameter ``U``: ', squeeze=False,
                                 transpose=transpose)

        # Algorithm: to integrate from time 0 to time dt, with linear
        # interpolation between inputs u(0) = u0 and u(dt) = u1, we solve
        #   xdot = A x + B u,        x(0) = x0
        #   udot = (u1 - u0) / dt,   u(0) = u0.

        # Solution is
        #   [ x(dt) ]       [ A*dt  B*dt  0 ] [  x0   ]
        #   [ u(dt) ] = exp [  0     0    I ] [  u0   ]
        #   [u1 - u0]       [  0     0    0 ] [u1 - u0]

        # Laço principal
        for i in range(1, n_steps):
            # divide o tempo de simulação em 9 partes para percorrer o vetor d_t
            if i < n_steps * 0.1:
                aux = 0
            elif i == n_steps * 0.1:
                aux = 1
            elif i == n_steps * 0.2:
                aux = 2
            elif i == n_steps * 0.3:
                aux = 3
            elif i == n_steps * 0.4:
                aux = 4
            elif i == n_steps * 0.5:
                aux = 5
            elif i == n_steps * 0.6:
                aux = 6
            elif i == n_steps * 0.7:
                aux = 7
            elif i == n_steps * 0.8:
                aux = 8
            elif i == n_steps * 0.9:
                aux = 9

            Il = (self._Vin / self._R) * (d_t[aux] / (1 - d_t[aux]))
            Vc = self._Vin * (d_t[aux] / (1-d_t[aux]))

            g11 = (self._Vin + Vc)/self._L
            g21 = -Vc/self._C

            sig111 = (g11 - b112) / (b111 - b112)
            sig112 = (b111 - g11) / (b111 - b112)
            eps211 = (g21 - b212) / (b211 - b212)
            eps212 = (b211 - g21) / (b211 - b212)

            alpha1 = eps211 * sig111
            alpha2 = eps211 * sig112
            alpha3 = eps212 * sig111
            alpha4 = eps212 * sig112

            B = np.asarray(somar(alpha4 * B4, somar(alpha3 * B3, somar(alpha1 * B1, alpha2 * B2))))

            U = d_t[aux] * np.ones_like(T)
            if U is not None:
                U = np.asarray(U)
            if len(U.shape) == 1:
                U = U.reshape(1, -1)

            M = np.block([[A * dt, B * dt, np.zeros((n_states, n_inputs))],
                          [np.zeros((n_inputs, n_states + n_inputs)),
                           np.identity(n_inputs)],
                          [np.zeros((n_inputs, n_states + 2 * n_inputs))]])
            expM = sp.linalg.expm(M)
            Ad = expM[:n_states, :n_states]
            Bd1 = expM[:n_states, n_states + n_inputs:]
            Bd0 = expM[:n_states, n_states:n_states + n_inputs] - Bd1

            xout[:, i] = (np.dot(Ad, xout[:, i - 1]) + np.dot(Bd0, U[:, i - 1]) +
                          np.dot(Bd1, U[:, i]))
            yout = np.dot(C, xout) + np.dot(E, U)

        return yout





