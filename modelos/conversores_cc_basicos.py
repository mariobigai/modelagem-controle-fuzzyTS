import control as ctr
from modelos import auxiliar
import numpy as np
import scipy as sp

class Buck_Boost_ideal_saida:
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
        Il_min = (self._Vin / self._R) * (d_min / pow((1 - d_min),2))
        Vc_min = self._Vin * (d_min / (1-d_min))

        # Região máxima de operação
        Il_max = (self._Vin / self._R) * (d_max / pow((1 - d_max),2))
        Vc_max = self._Vin * (d_max / (1-d_max))

        # Máximos e mínimos de g11
        b111 = (self._Vin + Vc_max) / self._L
        b112 = (self._Vin + Vc_min) / self._L

        # Máximos e mínimos de g12
        b211 = (-1 * Il_min) / self._C
        b212 = (-1 * Il_max) / self._C

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
            Il = (self._Vin / self._R) * (d_t[i] / pow((1 - d_t[i]),2))
            Vc = self._Vin * (d_t[i] / (1-d_t[i]))

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

            B = alpha1*B1+alpha2*B2+alpha3*B3+alpha4*B4

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


class Boost_ideal_saida:
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

        #Máximos e mínimos de f12
        a121 = -1/self._L + self._Vin/(self._L*Vc_max)
        a122 = -1/self._L + self._Vin/(self._L*Vc_min)
        print(a121, a122)

        # Máximos e mínimos de g11
        b111 = Vc_max/self._L
        b112 = Vc_min/self._L
        print(b111, b112)

        # Máximos e mínimos de g12
        b211 = -Il_max/self._C
        b212 = -Il_min/self._C
        print(b211, b212)

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
