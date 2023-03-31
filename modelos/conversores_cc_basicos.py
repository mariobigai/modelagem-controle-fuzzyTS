from auxiliar import *
class Buck_Boost_ideal:
    def __init__(self, R, C, L, Vin):
        self._R = R
        self._C = C
        self._L = L
        self._Vin = Vin

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

    def cria_SS:
        pass

    def forced_response_TS(sys, T=None, U=0., X0=0., transpose=False, interpolate=False, return_x=None, squeeze=None):

        # Pega as matrizes de estado
        A, B, C, D = np.asarray(sys.A), np.asarray(sys.B), np.asarray(sys.C), np.asarray(sys.D)

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

        # Erro se T é nulo
        if T is None:
            raise ValueError('Parameter ``T``: must be array-like, and contain '
                             '(strictly monotonic) increasing numbers.')

        # Verifica e atribui o T no formato correto
        T = _check_convert_array(T, [('any',), (1, 'any')], 'Parameter ``T``: ', squeeze=True, transpose=transpose)

        dt = T[1] - T[0] #Passo de simulação

        # Erro se T não for igualmente espaçado - Fixed-step size
        if not np.allclose(T[1:] - T[:-1], dt):
            raise ValueError("Parameter ``T``: time values must be "
                             "equally spaced.")

        n_steps = T.shape[0]  # número de passos de simulação

        # Cria vetor X0 se não for dado e testa se está no formato correto
        X0 = _check_convert_array(X0, [(n_states,), (n_states, 1)], 'Parameter ``X0``: ', squeeze=True)

        xout = np.zeros((n_states, n_steps))
        xout[:, 0] = X0
        yout = np.zeros((n_outputs, n_steps))

        ## Considerando apenas o caso contínuo
        legal_shapes = [(n_steps,), (1, n_steps)] if n_inputs == 1 else \
            [(n_inputs, n_steps)]

        U = _check_convert_array(U, legal_shapes,
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
            pass
        xout = 0
        yout = 0

        tout = T

        xout = np.transpose(xout)
        yout = np.transpose(yout)
    return xout, yout





