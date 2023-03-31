"""
Arquivo main para testar códigos dos modelos dos conversores
Trabalho Futuro - Refatorar implementando TDD
"""

import conversores_cc_basicos

conversor = conversores_cc_basicos.Buck_Boost_ideal(1.96, 375e-6, 0.146e-3, 24)
print(conversor)
