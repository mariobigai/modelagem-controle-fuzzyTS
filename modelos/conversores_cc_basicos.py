import control as ctr
import auxiliar
import numpy as np
import scipy as sp

class Buck_R:
    pass

class Boost_R:
    def __init__(self, R, C, L, Vin):
        self._R = R
        self._C = C
        self._L = L
        self._Vin = Vin

    def forced_response_TS(self, d_t, d_min, d_max, T=None, U=0., X0=0., transpose=False, interpolate=False, return_x=None, squeeze=None):

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
        Il_min = self._Vin/(pow((1 - d_min),2)*self._R)
        Vc_min = self._Vin/(1-d_min)

        # Região máxima de operação
        Il_max = self._Vin/(pow((1 - d_max),2)*self._R)
        Vc_max = self._Vin/(1-d_max)

        print(f'Il_min: {Il_min}; Il_max: {Il_max}')
        print(f'Vc_min: {Vc_min}; Vc_max: {Vc_max}')

        #Máximos e mínimos de f12
        a121 = -1/self._L + self._Vin/(self._L*Vc_max)
        a122 = -1/self._L + self._Vin/(self._L*Vc_min)
        print(f'f12_min: {a121}; f12_max: {a122}')

        # Máximos e mínimos de g11
        b111 = Vc_max/self._L
        b112 = Vc_min/self._L
        print(f'g11_min: {b111}; g11_max: {b112}')

        # Máximos e mínimos de g12
        b211 = -Il_max/self._C
        b212 = -Il_min/self._C
        print(f'g11_min: {b111}; g11_max: {b112}')

        # Modelos lineares locais
        A1 = np.array([[0, a121],[1/self._C, -1/(self._R*self._C)]])
        A3 = A1; A5=A1; A7=A1
        A2 = np.array([[0, a122], [1/self._C, -1/(self._R*self._C)]])
        A4 = A2; A6 = A2; A8 = A2

        B1 = np.array([[b111], [b211]])
        B2=B1
        B3 = np.array([[b111], [b212]])
        B4=B3
        B5 = np.array([[b112], [b211]])
        B6=B5
        B7 = np.array([[b112], [b212]])
        B8=B7

        # Considerando o inicialmente alpha1 = 1, demais=0
        A_TS = A8
        B_TS = B8
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

        #-------------------------------------------------------------------------------------------------------------
        # Laço principal
        for i in range(1, n_steps):
            Il = (self._Vin)/(pow((1 - d_t[i]),2)*self._R)
            Vc = self._Vin/(1-d_t[i])

            if Vc == 0:
                f12 = (-1/self._L) + (self._Vin/(0.01*self._L))
            else:
                f12 = (-1/self._L) + (self._Vin /(Vc * self._L))

            g11 = Vc/self._L

            g21 = -Il/self._C

            sig121 = (f12-a122)/(a121-a122)
            sig122 = (a121-f12)/(a121-a122)

            gamma111 = (g11-b112)/(b111-b112)
            gamma112 = (b111 - g11)/(b111 - b112)

            rho211 = (g21-b212)/(b211-b212)
            rho212 = (b211-g21)/(b211-b212)

            alpha1 = gamma111*rho211*sig121
            alpha2 = gamma111*rho211*sig122
            alpha3 = gamma111*rho212*sig121
            alpha4 = gamma111*rho212*sig122
            alpha5 = gamma112*rho211*sig121
            alpha6 = gamma112*rho211*sig122
            alpha7 = gamma112*rho212*sig121
            alpha8 = gamma112*rho212*sig122

            A = alpha1*A1 + alpha2*A2 + alpha3*A3 + alpha4*A4 + alpha5*A5 + alpha6*A6 + alpha7*A7 + alpha8*A8
            B = alpha1*B1 + alpha2*B2 + alpha3*B3 + alpha4*B4 + alpha5*B5 + alpha6*B6 + alpha7*B7 + alpha8*B8

            U = d_t[i] * np.ones_like(T)
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

class BuckBoost_R:
    def __init__(self, R, C, L, Vin):
        self._R = R
        self._C = C
        self._L = L
        self._Vin = Vin

    def forced_response_TS(self, d_t, d_min, d_max, il_simu, vo_simu, T=None, U=0., X0=0., transpose=False, interpolate=False, return_x=None, squeeze=None):

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
        #Região mínima de operação
        Il_min = (self._Vin / self._R) * (d_min / pow((1 - d_min),2))
        Vc_min = self._Vin * (d_min / (1-d_min))

        # # Região máxima de operação
        Il_max = (self._Vin / self._R) * (d_max / pow((1 - d_max),2))
        Vc_max = self._Vin * (d_max / (1-d_max))

        print(f'Il_min: {Il_min}; Il_max: {Il_max}')
        print(f'Vc_min: {Vc_min}; Vc_max: {Vc_max}')

        # Máximos e mínimos de g11
        b111 = (self._Vin + Vc_max)/self._L
        b112 = (self._Vin + Vc_min)/self._L
        print(f'g11_min: {b112}; g11_max: {b111}')

        # Máximos e mínimos de g12
        b211 = 0
        b212 = -Il_max/self._C
        print(f'g21_min: {b212}; g21_max: {b211}')

        # Modelos lineares locais
        B1 = np.array([[b111], [b211]])
        B2 = np.array([[b112], [b211]])
        B3 = np.array([[b111], [b212]])
        B4 = np.array([[b112], [b212]])

        # Considerando o inicialmente alpha1 = 1, alpha2,3,4 =0
        A_TS = [[0, -1/self._L],
                [1/self._C, -1/(self._R*self._C)]]
        B_TS = 0*B1+0*B2+0*B3+1*B4
        C_TS = [[0,1],[1,0]]
        E_TS = [[0],[0]]
        SS_TS = ctr.StateSpace(A_TS, B_TS, C_TS, E_TS)

        # Pega as matrizes de estado
        A, B, C, E = np.asarray(SS_TS.A), np.asarray(SS_TS.B), np.asarray(SS_TS.C), np.asarray(SS_TS.D)

        # ## ------------------------------------------------------------------------------------------------------------
        # # Verificações

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

        #-------------------------------------------------------------------------------------------------------------
        # Laço principal
        for i in range(1, n_steps):
            Il = il_simu[i]
            Vc = vo_simu[i]

            g11 = (self._Vin + Vc)/self._L
            g21 = -Il/self._C

            sig111 = (g11 - b112) / (b111 - b112)
            sig112 = (b111 - g11) / (b111 - b112)

            eps211 = (g21 - b212) / (b211 - b212)
            eps212 = (b211 - g21) / (b211 - b212)

            alpha1 = eps211 * sig111
            alpha2 = eps211 * sig112
            alpha3 = eps212 * sig111
            alpha4 = eps212 * sig112

            h1 = alpha1/(alpha1+alpha2+alpha3+alpha4)
            h2 = alpha2/(alpha1+alpha2+alpha3+alpha4)
            h3 = alpha3/(alpha1+alpha2+alpha3+alpha4)
            h4 = alpha4/(alpha1+alpha2+alpha3+alpha4)

            B = h1*B1+h2*B2+h3*B3+h4*B4

            print(f'{i}/{n_steps}')
            U = d_t[i] * np.ones_like(T)
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

class Buck_PV:
    def __init__(self, Vo, L, C, Iin, Rp):
        self._Vo = Vo
        self._C = C
        self._L = L
        self._Iin = Iin
        self._Rp = Rp
    def forced_response_TS(self, d_t, d_min, d_max, il_simu, vcin_simu, T=None, U=0., X0=0., transpose=False):

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
        # máximos e mínimos considerando os valores genéricos
        # # a11 = -2400; a12 = -6000
        # a11 = -342.8571; a12 = -857.1429 #Vo=12, L=700e-6, C=500e-6, Iin=10, Rp=10
        # a21 = 1500.0038; a22 = 960.0015
        # # b31 = 500e3; b32 = 200e3
        # b31 = 29761.8571; b32 = 19047.5743 #Vo=12, L=700e-6, C=500e-6, Iin=10, Rp=10
        # b41 = -26666.6666; b42 = -41666.6666

        # # máximos e mínimos considerando os valores de projeto
        a11 = -10855.3078 ; a12 = -27138.2695
        a21 = 533.3367; a22 = 296.2963
        b31 = 2261522.4570; b32 = 904608.9828
        b41 = -1929; b42 = -3472.2222

        print(f'min(z1) = a12 = {a12} max(z1) = a11 = {a11}')
        print(f'min(z2) = a22 = {a22} max(z2) = a21 = {a21}')
        print(f'min(z3) = b32 = {b32} max(z3) = b31 = {b31}')
        print(f'min(z4) = b42 = {b42} max(z4) = b41 = {b41}')

        # Modelos Lineares Locais
        A1 = np.array([[0, a11], [a21, -1/(self._Rp*self._C)]])
        A5 = A1; A9 = A1; A13 = A1

        A2 = np.array([[0, a12], [a21, -1/(self._Rp*self._C)]])
        A6 = A2; A10 = A2; A14 = A2

        A3 = np.array([[0, a11], [a22, -1/(self._Rp * self._C)]])
        A7 = A3; A11 = A3; A15 = A3

        A4 = np.array([[0, a12], [a22, -1/(self._Rp * self._C)]])
        A8 = A4; A12 = A4; A16 = A4

        B1 = np.array([[b31], [b41]])
        B2=B1; B3=B1; B4=B1

        B5 = np.array([[b32], [b41]])
        B6 = B5; B7 = B5; B8 = B5

        B9 = np.array([[b31], [b42]])
        B10 = B9; B11 = B9; B12 = B9

        B13 = np.array([[b32], [b42]])
        B14 = B13; B15 = B13; B16 = B13

        #Considerando inicialmente alpha16=1 demais = 0
        A_TS = A16
        B_TS = B16
        C_TS = np.array([[0, 1], [1, 0]])
        E_TS = np.array([[0], [0]])
        SS_TS = ctr.StateSpace(A_TS, B_TS, C_TS, E_TS)

        # Pega as matrizes de estado
        A, B, C, E = np.asarray(SS_TS.A), np.asarray(SS_TS.B), np.asarray(SS_TS.C), np.asarray(SS_TS.D)

        ## ------------------------------------------------------------------------------------------------------------
        # Verificações

        # Numero de estados
        n_states = A.shape[0]

        # Numero de Entradas
        n_inputs = B.shape[1]

        # Numero de Saídas
        n_outputs = C.shape[0]

        # Converte as entradas e tempo de simulação em numpy arrays
        if U is not None:
            U = np.asarray(U)  # cria array de entrada
        if T is not None:
            T = np.asarray(T)  # cria array de tempo

        # Garante que U e T tem o mesmo tamanho
        if (U.ndim == 1 and U.shape[0] != T.shape[0]) or \
                (U.ndim > 1 and U.shape[1] != T.shape[0]):
            ValueError('Pamameter ``T`` must have same elements as'
                       ' the number of columns in input array ``U``')

        # Erro se T é nulo
        if T is None:
            raise ValueError('Parameter ``T``: must be array-like, and contain '
                             '(strictly monotonic) increasing numbers.')

        # Verifica e atribui o T no formato correto
        T = auxiliar._check_convert_array(T, [('any',), (1, 'any')], 'Parameter ``T``: ', squeeze=True,
                                          transpose=transpose)

        dt = T[1] - T[0]  # Passo de simulação

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

        # -------------------------------------------------------------------------------------------------------------
        # Laço principal
        for i in range(1, n_steps):
            Il = il_simu[i]
            Vcin = vcin_simu[i]

            if Vcin == 0:
                z1 = -self._Vo / (self._L * 0.01)
            else:
                z1 = -self._Vo / (self._L * Vcin)

            if Il == 0:
                z2 = self._Iin / (self._C * 0.01)
            else:
                z2 = self._Iin / (self._C * Il)

            z3 = Vcin / self._L

            z4 = -Il / self._C

            M11 = (z1 - a12) / (a11 - a12)
            M12 = (a11 - z1) / (a11 - a12)

            M21 = (z2 - a22) / (a21 - a22)
            M22 = (a21 - z2) / (a21 - a22)

            M31 = (z3 - b32) / (b31 - b32)
            M32 = (b31 - z3) / (b31 - b32)

            M41 = (z4 - b42) / (b41 - b42)
            M42 = (b41 - z4) / (b41 - b42)

            W1 = M41*M31*M21*M11; W9 = M42*M31*M21*M11;
            W2 = M41*M31*M21*M12; W10 = M42*M31*M21*M12;
            W3 = M41*M31*M22*M11; W11 = M42*M31*M22*M11;
            W4 = M41*M31*M22*M12; W12 = M42*M31*M22*M12;
            W5 = M41*M32*M21*M11; W13 = M42*M32*M21*M11;
            W6 = M41*M32*M21*M12; W14 = M42*M32*M21*M12;
            W7 = M41*M32*M22*M11; W15 = M42*M32*M22*M11;
            W8 = M41*M32*M22*M12; W16 = M42*M32*M22*M12;

            h1 = W1/(W1+W2+W3+W4+W5+W6+W7+W8+W9+W10+W11+W12+W13+W14+W15+W16)
            h2 = W2 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)
            h3 = W3 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)
            h4 = W4 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)
            h5 = W5 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)
            h6 = W6 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)
            h7 = W7 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)
            h8 = W8 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)
            h9 = W9 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)
            h10 = W10 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)
            h11 = W11 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)
            h12 = W12 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)
            h13 = W13 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)
            h14 = W14 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)
            h15 = W15 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)
            h16 = W16 / (W1 + W2 + W3 + W4 + W5 + W6 + W7 + W8 + W9 + W10 + W11 + W12 + W13 + W14 + W15 + W16)

            A = h1 * A1 + h2 * A2 + h3 * A3 + h4 * A4+ h5 * A5 + h6 * A6 + h7 * A7 + h8 * A8+ h9 * A9 + h10 * A10 + h11 * A11 + h12 * A12 + h13 * A13 + h14 * A14 + h15 * A15 + h16 * A16
            B = h1 * B1 + h2 * B2 + h3 * B3 + h4 * B4 + h5 * B5 + h6 * B6 + h7 * B7 + h8 * B8 + h9 * B9 + h10 * B10 + h11 * B11 + h12 * B12 + h13 * B13 + h14 * B14 + h15 * B15 + h16 * B16

            U = d_t[i] * np.ones_like(T)
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

class Boost_PV:
    pass

class BuckBoost_PV:
    pass
