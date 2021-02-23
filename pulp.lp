\* Modelname *\
Minimize
OBJ: - 0.001 Charge_at_node_0_Drone_0 - 0.001 Charge_at_node_1_Drone_0
 - 0.001 Charge_at_node_2_Drone_0 - 0.001 Charge_at_node_3_Drone_0
 - 0.001 Charge_at_node_4_Drone_0 - 0.001 Charge_at_node_5_Drone_0
 - 0.001 Charge_at_node_6_Drone_0 - 0.001 Charge_at_node_7_Drone_0
 + Link0_1_Drone_0 + 2 Link0_2_Drone_0 + Link0_3_Drone_0
 + 1.41421356237 Link0_4_Drone_0 + 2.2360679775 Link0_5_Drone_0
 + 11.1803398875 Link0_6_Drone_0 + 11.1803398875 Link0_7_Drone_0
 + Link1_0_Drone_0 + Link1_2_Drone_0 + 1.41421356237 Link1_3_Drone_0
 + Link1_4_Drone_0 + 1.41421356237 Link1_5_Drone_0
 + 10.7703296143 Link1_6_Drone_0 + 10.7703296143 Link1_7_Drone_0
 + 2 Link2_0_Drone_0 + Link2_1_Drone_0 + 2.2360679775 Link2_3_Drone_0
 + 1.41421356237 Link2_4_Drone_0 + Link2_5_Drone_0
 + 10.4403065089 Link2_6_Drone_0 + 10.4403065089 Link2_7_Drone_0
 + Link3_0_Drone_0 + 1.41421356237 Link3_1_Drone_0
 + 2.2360679775 Link3_2_Drone_0 + Link3_4_Drone_0 + 2 Link3_5_Drone_0
 + 10.295630141 Link3_6_Drone_0 + 10.295630141 Link3_7_Drone_0
 + 1.41421356237 Link4_0_Drone_0 + Link4_1_Drone_0
 + 1.41421356237 Link4_2_Drone_0 + Link4_3_Drone_0 + Link4_5_Drone_0
 + 9.8488578018 Link4_6_Drone_0 + 9.8488578018 Link4_7_Drone_0
 + 2.2360679775 Link5_0_Drone_0 + 1.41421356237 Link5_1_Drone_0
 + Link5_2_Drone_0 + 2 Link5_3_Drone_0 + Link5_4_Drone_0
 + 9.48683298051 Link5_6_Drone_0 + 9.48683298051 Link5_7_Drone_0
 + 11.1803398875 Link6_0_Drone_0 + 10.7703296143 Link6_1_Drone_0
 + 10.4403065089 Link6_2_Drone_0 + 10.295630141 Link6_3_Drone_0
 + 9.8488578018 Link6_4_Drone_0 + 9.48683298051 Link6_5_Drone_0
 + 11.1803398875 Link7_0_Drone_0 + 10.7703296143 Link7_1_Drone_0
 + 10.4403065089 Link7_2_Drone_0 + 10.295630141 Link7_3_Drone_0
 + 9.8488578018 Link7_4_Drone_0 + 9.48683298051 Link7_5_Drone_0
 + 2.67 Time_at_node_0_Drone_0 + 2.67 Time_at_node_1_Drone_0
 + 2.67 Time_at_node_2_Drone_0 + 2.67 Time_at_node_3_Drone_0
 + 2.67 Time_at_node_4_Drone_0 + 2.67 Time_at_node_5_Drone_0
 + 2.67 Time_at_node_6_Drone_0 + 2.67 Time_at_node_7_Drone_0
Subject To
_C1: - Link0_1_Drone_0 - Link0_2_Drone_0 - Link0_3_Drone_0 - Link0_4_Drone_0
 - Link0_5_Drone_0 - Link0_6_Drone_0 - Link0_7_Drone_0 + Link1_0_Drone_0
 + Link2_0_Drone_0 + Link3_0_Drone_0 + Link4_0_Drone_0 + Link5_0_Drone_0
 + Link6_0_Drone_0 + Link7_0_Drone_0 = 0
_C10: Link2_0_Drone_0 + Link2_1_Drone_0 + Link2_3_Drone_0 + Link2_4_Drone_0
 + Link2_5_Drone_0 + Link2_6_Drone_0 + Link2_7_Drone_0 = 1
_C100: - 1.32 Charge_at_node_1_Drone_0 + 1000.857 Link1_4_Drone_0
 + Time_at_node_1_Drone_0 - Time_at_node_4_Drone_0 = 860
_C101: 1001.21198102 Link1_5_Drone_0 + Time_at_node_1_Drone_0
 - Time_at_node_5_Drone_0 = 985
_C102: - 1.32 Charge_at_node_1_Drone_0 + 1001.21198102 Link1_5_Drone_0
 + Time_at_node_1_Drone_0 - Time_at_node_5_Drone_0 = 860
_C103: 1000.857 Link2_1_Drone_0 - Time_at_node_1_Drone_0
 + Time_at_node_2_Drone_0 = 985
_C104: - 1.32 Charge_at_node_2_Drone_0 + 1000.857 Link2_1_Drone_0
 - Time_at_node_1_Drone_0 + Time_at_node_2_Drone_0 = 860
_C105: 1001.91631026 Link2_3_Drone_0 + Time_at_node_2_Drone_0
 - Time_at_node_3_Drone_0 = 985
_C106: - 1.32 Charge_at_node_2_Drone_0 + 1001.91631026 Link2_3_Drone_0
 + Time_at_node_2_Drone_0 - Time_at_node_3_Drone_0 = 860
_C107: 1001.21198102 Link2_4_Drone_0 + Time_at_node_2_Drone_0
 - Time_at_node_4_Drone_0 = 985
_C108: - 1.32 Charge_at_node_2_Drone_0 + 1001.21198102 Link2_4_Drone_0
 + Time_at_node_2_Drone_0 - Time_at_node_4_Drone_0 = 860
_C109: 1000.857 Link2_5_Drone_0 + Time_at_node_2_Drone_0
 - Time_at_node_5_Drone_0 = 985
_C11: Link3_0_Drone_0 + Link3_1_Drone_0 + Link3_2_Drone_0 + Link3_4_Drone_0
 + Link3_5_Drone_0 + Link3_6_Drone_0 + Link3_7_Drone_0 = 1
_C110: - 1.32 Charge_at_node_2_Drone_0 + 1000.857 Link2_5_Drone_0
 + Time_at_node_2_Drone_0 - Time_at_node_5_Drone_0 = 860
_C111: 1001.21198102 Link3_1_Drone_0 - Time_at_node_1_Drone_0
 + Time_at_node_3_Drone_0 = 985
_C112: - 1.32 Charge_at_node_3_Drone_0 + 1001.21198102 Link3_1_Drone_0
 - Time_at_node_1_Drone_0 + Time_at_node_3_Drone_0 = 860
_C113: 1001.91631026 Link3_2_Drone_0 - Time_at_node_2_Drone_0
 + Time_at_node_3_Drone_0 = 985
_C114: - 1.32 Charge_at_node_3_Drone_0 + 1001.91631026 Link3_2_Drone_0
 - Time_at_node_2_Drone_0 + Time_at_node_3_Drone_0 = 860
_C115: 1000.857 Link3_4_Drone_0 + Time_at_node_3_Drone_0
 - Time_at_node_4_Drone_0 = 985
_C116: - 1.32 Charge_at_node_3_Drone_0 + 1000.857 Link3_4_Drone_0
 + Time_at_node_3_Drone_0 - Time_at_node_4_Drone_0 = 860
_C117: 1001.714 Link3_5_Drone_0 + Time_at_node_3_Drone_0
 - Time_at_node_5_Drone_0 = 985
_C118: - 1.32 Charge_at_node_3_Drone_0 + 1001.714 Link3_5_Drone_0
 + Time_at_node_3_Drone_0 - Time_at_node_5_Drone_0 = 860
_C119: 1000.857 Link4_1_Drone_0 - Time_at_node_1_Drone_0
 + Time_at_node_4_Drone_0 = 985
_C12: Link4_0_Drone_0 + Link4_1_Drone_0 + Link4_2_Drone_0 + Link4_3_Drone_0
 + Link4_5_Drone_0 + Link4_6_Drone_0 + Link4_7_Drone_0 = 1
_C120: - 1.32 Charge_at_node_4_Drone_0 + 1000.857 Link4_1_Drone_0
 - Time_at_node_1_Drone_0 + Time_at_node_4_Drone_0 = 860
_C121: 1001.21198102 Link4_2_Drone_0 - Time_at_node_2_Drone_0
 + Time_at_node_4_Drone_0 = 985
_C122: - 1.32 Charge_at_node_4_Drone_0 + 1001.21198102 Link4_2_Drone_0
 - Time_at_node_2_Drone_0 + Time_at_node_4_Drone_0 = 860
_C123: 1000.857 Link4_3_Drone_0 - Time_at_node_3_Drone_0
 + Time_at_node_4_Drone_0 = 985
_C124: - 1.32 Charge_at_node_4_Drone_0 + 1000.857 Link4_3_Drone_0
 - Time_at_node_3_Drone_0 + Time_at_node_4_Drone_0 = 860
_C125: 1000.857 Link4_5_Drone_0 + Time_at_node_4_Drone_0
 - Time_at_node_5_Drone_0 = 985
_C126: - 1.32 Charge_at_node_4_Drone_0 + 1000.857 Link4_5_Drone_0
 + Time_at_node_4_Drone_0 - Time_at_node_5_Drone_0 = 860
_C127: 1001.21198102 Link5_1_Drone_0 - Time_at_node_1_Drone_0
 + Time_at_node_5_Drone_0 = 985
_C128: - 1.32 Charge_at_node_5_Drone_0 + 1001.21198102 Link5_1_Drone_0
 - Time_at_node_1_Drone_0 + Time_at_node_5_Drone_0 = 860
_C129: 1000.857 Link5_2_Drone_0 - Time_at_node_2_Drone_0
 + Time_at_node_5_Drone_0 = 985
_C13: Link5_0_Drone_0 + Link5_1_Drone_0 + Link5_2_Drone_0 + Link5_3_Drone_0
 + Link5_4_Drone_0 + Link5_6_Drone_0 + Link5_7_Drone_0 = 1
_C130: - 1.32 Charge_at_node_5_Drone_0 + 1000.857 Link5_2_Drone_0
 - Time_at_node_2_Drone_0 + Time_at_node_5_Drone_0 = 860
_C131: 1001.714 Link5_3_Drone_0 - Time_at_node_3_Drone_0
 + Time_at_node_5_Drone_0 = 985
_C132: - 1.32 Charge_at_node_5_Drone_0 + 1001.714 Link5_3_Drone_0
 - Time_at_node_3_Drone_0 + Time_at_node_5_Drone_0 = 860
_C133: 1000.857 Link5_4_Drone_0 - Time_at_node_4_Drone_0
 + Time_at_node_5_Drone_0 = 985
_C134: - 1.32 Charge_at_node_5_Drone_0 + 1000.857 Link5_4_Drone_0
 - Time_at_node_4_Drone_0 + Time_at_node_5_Drone_0 = 860
_C135: 1009.23017248 Link6_1_Drone_0 - Time_at_node_1_Drone_0
 + Time_at_node_6_Drone_0 = 985
_C136: - 1.32 Charge_at_node_6_Drone_0 + 1009.23017248 Link6_1_Drone_0
 - Time_at_node_1_Drone_0 + Time_at_node_6_Drone_0 = 860
_C137: 1008.94734268 Link6_2_Drone_0 - Time_at_node_2_Drone_0
 + Time_at_node_6_Drone_0 = 985
_C138: - 1.32 Charge_at_node_6_Drone_0 + 1008.94734268 Link6_2_Drone_0
 - Time_at_node_2_Drone_0 + Time_at_node_6_Drone_0 = 860
_C139: 1008.82335503 Link6_3_Drone_0 - Time_at_node_3_Drone_0
 + Time_at_node_6_Drone_0 = 985
_C14: Link0_1_Drone_0 + Link0_2_Drone_0 + Link0_3_Drone_0 + Link0_4_Drone_0
 + Link0_5_Drone_0 + Link0_6_Drone_0 + Link0_7_Drone_0 = 1
_C140: - 1.32 Charge_at_node_6_Drone_0 + 1008.82335503 Link6_3_Drone_0
 - Time_at_node_3_Drone_0 + Time_at_node_6_Drone_0 = 860
_C141: 1008.44047114 Link6_4_Drone_0 - Time_at_node_4_Drone_0
 + Time_at_node_6_Drone_0 = 985
_C142: - 1.32 Charge_at_node_6_Drone_0 + 1008.44047114 Link6_4_Drone_0
 - Time_at_node_4_Drone_0 + Time_at_node_6_Drone_0 = 860
_C143: 1008.13021586 Link6_5_Drone_0 - Time_at_node_5_Drone_0
 + Time_at_node_6_Drone_0 = 985
_C144: - 1.32 Charge_at_node_6_Drone_0 + 1008.13021586 Link6_5_Drone_0
 - Time_at_node_5_Drone_0 + Time_at_node_6_Drone_0 = 860
_C145: 1009.23017248 Link7_1_Drone_0 - Time_at_node_1_Drone_0
 + Time_at_node_7_Drone_0 = 985
_C146: - 1.32 Charge_at_node_7_Drone_0 + 1009.23017248 Link7_1_Drone_0
 - Time_at_node_1_Drone_0 + Time_at_node_7_Drone_0 = 860
_C147: 1008.94734268 Link7_2_Drone_0 - Time_at_node_2_Drone_0
 + Time_at_node_7_Drone_0 = 985
_C148: - 1.32 Charge_at_node_7_Drone_0 + 1008.94734268 Link7_2_Drone_0
 - Time_at_node_2_Drone_0 + Time_at_node_7_Drone_0 = 860
_C149: 1008.82335503 Link7_3_Drone_0 - Time_at_node_3_Drone_0
 + Time_at_node_7_Drone_0 = 985
_C15: Charge_at_node_0_Drone_0 = 100
_C150: - 1.32 Charge_at_node_7_Drone_0 + 1008.82335503 Link7_3_Drone_0
 - Time_at_node_3_Drone_0 + Time_at_node_7_Drone_0 = 860
_C151: 1008.44047114 Link7_4_Drone_0 - Time_at_node_4_Drone_0
 + Time_at_node_7_Drone_0 = 985
_C152: - 1.32 Charge_at_node_7_Drone_0 + 1008.44047114 Link7_4_Drone_0
 - Time_at_node_4_Drone_0 + Time_at_node_7_Drone_0 = 860
_C153: 1008.13021586 Link7_5_Drone_0 - Time_at_node_5_Drone_0
 + Time_at_node_7_Drone_0 = 985
_C154: - 1.32 Charge_at_node_7_Drone_0 + 1008.13021586 Link7_5_Drone_0
 - Time_at_node_5_Drone_0 + Time_at_node_7_Drone_0 = 860
_C16: Charge_at_node_6_Drone_0 = 100
_C17: Charge_at_node_7_Drone_0 = 100
_C18: Charge_at_node_0_Drone_0 = 5
_C19: Charge_at_node_1_Drone_0 = 5
_C2: Link0_1_Drone_0 - Link1_0_Drone_0 - Link1_2_Drone_0 - Link1_3_Drone_0
 - Link1_4_Drone_0 - Link1_5_Drone_0 - Link1_6_Drone_0 - Link1_7_Drone_0
 + Link2_1_Drone_0 + Link3_1_Drone_0 + Link4_1_Drone_0 + Link5_1_Drone_0
 + Link6_1_Drone_0 + Link7_1_Drone_0 = 0
_C20: Charge_at_node_2_Drone_0 = 5
_C21: Charge_at_node_3_Drone_0 = 5
_C22: Charge_at_node_4_Drone_0 = 5
_C23: Charge_at_node_5_Drone_0 = 5
_C24: Charge_at_node_6_Drone_0 = 5
_C25: Charge_at_node_7_Drone_0 = 5
_C26: Charge_at_node_0_Drone_0 - Charge_at_node_1_Drone_0
 - 1001.43 Link0_1_Drone_0 = -999.99
_C27: Charge_at_node_0_Drone_0 - Charge_at_node_2_Drone_0
 - 1002.86 Link0_2_Drone_0 = -999.99
_C28: Charge_at_node_0_Drone_0 - Charge_at_node_3_Drone_0
 - 1001.43 Link0_3_Drone_0 = -999.99
_C29: Charge_at_node_0_Drone_0 - Charge_at_node_4_Drone_0
 - 1002.02232539 Link0_4_Drone_0 = -999.99
_C3: Link0_2_Drone_0 + Link1_2_Drone_0 - Link2_0_Drone_0 - Link2_1_Drone_0
 - Link2_3_Drone_0 - Link2_4_Drone_0 - Link2_5_Drone_0 - Link2_6_Drone_0
 - Link2_7_Drone_0 + Link3_2_Drone_0 + Link4_2_Drone_0 + Link5_2_Drone_0
 + Link6_2_Drone_0 + Link7_2_Drone_0 = 0
_C30: Charge_at_node_0_Drone_0 - Charge_at_node_5_Drone_0
 - 1003.19757721 Link0_5_Drone_0 = -999.99
_C31: Charge_at_node_0_Drone_0 - 1015.98788604 Link0_6_Drone_0 = -1000
_C32: Charge_at_node_0_Drone_0 - 1015.98788604 Link0_7_Drone_0 = -1000
_C33: Charge_at_node_1_Drone_0 - 1001.43 Link1_0_Drone_0 = -1000
_C34: Charge_at_node_1_Drone_0 - Charge_at_node_2_Drone_0
 - 1001.43 Link1_2_Drone_0 = -999.99
_C35: Charge_at_node_1_Drone_0 - Charge_at_node_3_Drone_0
 - 1002.02232539 Link1_3_Drone_0 = -999.99
_C36: Charge_at_node_1_Drone_0 - Charge_at_node_4_Drone_0
 - 1001.43 Link1_4_Drone_0 = -999.99
_C37: Charge_at_node_1_Drone_0 - Charge_at_node_5_Drone_0
 - 1002.02232539 Link1_5_Drone_0 = -999.99
_C38: Charge_at_node_1_Drone_0 - 1015.40157135 Link1_6_Drone_0 = -1000
_C39: Charge_at_node_1_Drone_0 - 1015.40157135 Link1_7_Drone_0 = -1000
_C4: Link0_3_Drone_0 + Link1_3_Drone_0 + Link2_3_Drone_0 - Link3_0_Drone_0
 - Link3_1_Drone_0 - Link3_2_Drone_0 - Link3_4_Drone_0 - Link3_5_Drone_0
 - Link3_6_Drone_0 - Link3_7_Drone_0 + Link4_3_Drone_0 + Link5_3_Drone_0
 + Link6_3_Drone_0 + Link7_3_Drone_0 = 0
_C40: Charge_at_node_2_Drone_0 - 1002.86 Link2_0_Drone_0 = -1000
_C41: - Charge_at_node_1_Drone_0 + Charge_at_node_2_Drone_0
 - 1001.43 Link2_1_Drone_0 = -999.99
_C42: Charge_at_node_2_Drone_0 - Charge_at_node_3_Drone_0
 - 1003.19757721 Link2_3_Drone_0 = -999.99
_C43: Charge_at_node_2_Drone_0 - Charge_at_node_4_Drone_0
 - 1002.02232539 Link2_4_Drone_0 = -999.99
_C44: Charge_at_node_2_Drone_0 - Charge_at_node_5_Drone_0
 - 1001.43 Link2_5_Drone_0 = -999.99
_C45: Charge_at_node_2_Drone_0 - 1014.92963831 Link2_6_Drone_0 = -1000
_C46: Charge_at_node_2_Drone_0 - 1014.92963831 Link2_7_Drone_0 = -1000
_C47: Charge_at_node_3_Drone_0 - 1001.43 Link3_0_Drone_0 = -1000
_C48: - Charge_at_node_1_Drone_0 + Charge_at_node_3_Drone_0
 - 1002.02232539 Link3_1_Drone_0 = -999.99
_C49: - Charge_at_node_2_Drone_0 + Charge_at_node_3_Drone_0
 - 1003.19757721 Link3_2_Drone_0 = -999.99
_C5: Link0_4_Drone_0 + Link1_4_Drone_0 + Link2_4_Drone_0 + Link3_4_Drone_0
 - Link4_0_Drone_0 - Link4_1_Drone_0 - Link4_2_Drone_0 - Link4_3_Drone_0
 - Link4_5_Drone_0 - Link4_6_Drone_0 - Link4_7_Drone_0 + Link5_4_Drone_0
 + Link6_4_Drone_0 + Link7_4_Drone_0 = 0
_C50: Charge_at_node_3_Drone_0 - Charge_at_node_4_Drone_0
 - 1001.43 Link3_4_Drone_0 = -999.99
_C51: Charge_at_node_3_Drone_0 - Charge_at_node_5_Drone_0
 - 1002.86 Link3_5_Drone_0 = -999.99
_C52: Charge_at_node_3_Drone_0 - 1014.7227511 Link3_6_Drone_0 = -1000
_C53: Charge_at_node_3_Drone_0 - 1014.7227511 Link3_7_Drone_0 = -1000
_C54: Charge_at_node_4_Drone_0 - 1002.02232539 Link4_0_Drone_0 = -1000
_C55: - Charge_at_node_1_Drone_0 + Charge_at_node_4_Drone_0
 - 1001.43 Link4_1_Drone_0 = -999.99
_C56: - Charge_at_node_2_Drone_0 + Charge_at_node_4_Drone_0
 - 1002.02232539 Link4_2_Drone_0 = -999.99
_C57: - Charge_at_node_3_Drone_0 + Charge_at_node_4_Drone_0
 - 1001.43 Link4_3_Drone_0 = -999.99
_C58: Charge_at_node_4_Drone_0 - Charge_at_node_5_Drone_0
 - 1001.43 Link4_5_Drone_0 = -999.99
_C59: Charge_at_node_4_Drone_0 - 1014.08386666 Link4_6_Drone_0 = -1000
_C6: Link0_5_Drone_0 + Link1_5_Drone_0 + Link2_5_Drone_0 + Link3_5_Drone_0
 + Link4_5_Drone_0 - Link5_0_Drone_0 - Link5_1_Drone_0 - Link5_2_Drone_0
 - Link5_3_Drone_0 - Link5_4_Drone_0 - Link5_6_Drone_0 - Link5_7_Drone_0
 + Link6_5_Drone_0 + Link7_5_Drone_0 = 0
_C60: Charge_at_node_4_Drone_0 - 1014.08386666 Link4_7_Drone_0 = -1000
_C61: Charge_at_node_5_Drone_0 - 1003.19757721 Link5_0_Drone_0 = -1000
_C62: - Charge_at_node_1_Drone_0 + Charge_at_node_5_Drone_0
 - 1002.02232539 Link5_1_Drone_0 = -999.99
_C63: - Charge_at_node_2_Drone_0 + Charge_at_node_5_Drone_0
 - 1001.43 Link5_2_Drone_0 = -999.99
_C64: - Charge_at_node_3_Drone_0 + Charge_at_node_5_Drone_0
 - 1002.86 Link5_3_Drone_0 = -999.99
_C65: - Charge_at_node_4_Drone_0 + Charge_at_node_5_Drone_0
 - 1001.43 Link5_4_Drone_0 = -999.99
_C66: Charge_at_node_5_Drone_0 - 1013.56617116 Link5_6_Drone_0 = -1000
_C67: Charge_at_node_5_Drone_0 - 1013.56617116 Link5_7_Drone_0 = -1000
_C68: Charge_at_node_6_Drone_0 - 1015.98788604 Link6_0_Drone_0 = -1000
_C69: - Charge_at_node_1_Drone_0 + Charge_at_node_6_Drone_0
 - 1015.40157135 Link6_1_Drone_0 = -999.99
_C7: Link0_6_Drone_0 + Link1_6_Drone_0 + Link2_6_Drone_0 + Link3_6_Drone_0
 + Link4_6_Drone_0 + Link5_6_Drone_0 - Link6_0_Drone_0 - Link6_1_Drone_0
 - Link6_2_Drone_0 - Link6_3_Drone_0 - Link6_4_Drone_0 - Link6_5_Drone_0
 + 0 Link6_6_Drone_0 - Link6_7_Drone_0 + Link7_6_Drone_0 = 0
_C70: - Charge_at_node_2_Drone_0 + Charge_at_node_6_Drone_0
 - 1014.92963831 Link6_2_Drone_0 = -999.99
_C71: - Charge_at_node_3_Drone_0 + Charge_at_node_6_Drone_0
 - 1014.7227511 Link6_3_Drone_0 = -999.99
_C72: - Charge_at_node_4_Drone_0 + Charge_at_node_6_Drone_0
 - 1014.08386666 Link6_4_Drone_0 = -999.99
_C73: - Charge_at_node_5_Drone_0 + Charge_at_node_6_Drone_0
 - 1013.56617116 Link6_5_Drone_0 = -999.99
_C74: Charge_at_node_6_Drone_0 - 1000 Link6_6_Drone_0 = -1000
_C75: Charge_at_node_6_Drone_0 - 1000 Link6_7_Drone_0 = -1000
_C76: Charge_at_node_7_Drone_0 - 1015.98788604 Link7_0_Drone_0 = -1000
_C77: - Charge_at_node_1_Drone_0 + Charge_at_node_7_Drone_0
 - 1015.40157135 Link7_1_Drone_0 = -999.99
_C78: - Charge_at_node_2_Drone_0 + Charge_at_node_7_Drone_0
 - 1014.92963831 Link7_2_Drone_0 = -999.99
_C79: - Charge_at_node_3_Drone_0 + Charge_at_node_7_Drone_0
 - 1014.7227511 Link7_3_Drone_0 = -999.99
_C8: Link0_7_Drone_0 + Link1_7_Drone_0 + Link2_7_Drone_0 + Link3_7_Drone_0
 + Link4_7_Drone_0 + Link5_7_Drone_0 + Link6_7_Drone_0 - Link7_0_Drone_0
 - Link7_1_Drone_0 - Link7_2_Drone_0 - Link7_3_Drone_0 - Link7_4_Drone_0
 - Link7_5_Drone_0 - Link7_6_Drone_0 + 0 Link7_7_Drone_0 = 0
_C80: - Charge_at_node_4_Drone_0 + Charge_at_node_7_Drone_0
 - 1014.08386666 Link7_4_Drone_0 = -999.99
_C81: - Charge_at_node_5_Drone_0 + Charge_at_node_7_Drone_0
 - 1013.56617116 Link7_5_Drone_0 = -999.99
_C82: Charge_at_node_7_Drone_0 - 1000 Link7_6_Drone_0 = -1000
_C83: Charge_at_node_7_Drone_0 - 1000 Link7_7_Drone_0 = -1000
_C84: Time_at_node_0_Drone_0 = 0
_C85: 1000.857 Link0_1_Drone_0 + Time_at_node_0_Drone_0
 - Time_at_node_1_Drone_0 = 985
_C86: - 1.32 Charge_at_node_0_Drone_0 + 1000.857 Link0_1_Drone_0
 + Time_at_node_0_Drone_0 - Time_at_node_1_Drone_0 = 860
_C87: 1001.714 Link0_2_Drone_0 + Time_at_node_0_Drone_0
 - Time_at_node_2_Drone_0 = 985
_C88: - 1.32 Charge_at_node_0_Drone_0 + 1001.714 Link0_2_Drone_0
 + Time_at_node_0_Drone_0 - Time_at_node_2_Drone_0 = 860
_C89: 1000.857 Link0_3_Drone_0 + Time_at_node_0_Drone_0
 - Time_at_node_3_Drone_0 = 985
_C9: Link1_0_Drone_0 + Link1_2_Drone_0 + Link1_3_Drone_0 + Link1_4_Drone_0
 + Link1_5_Drone_0 + Link1_6_Drone_0 + Link1_7_Drone_0 = 1
_C90: - 1.32 Charge_at_node_0_Drone_0 + 1000.857 Link0_3_Drone_0
 + Time_at_node_0_Drone_0 - Time_at_node_3_Drone_0 = 860
_C91: 1001.21198102 Link0_4_Drone_0 + Time_at_node_0_Drone_0
 - Time_at_node_4_Drone_0 = 985
_C92: - 1.32 Charge_at_node_0_Drone_0 + 1001.21198102 Link0_4_Drone_0
 + Time_at_node_0_Drone_0 - Time_at_node_4_Drone_0 = 860
_C93: 1001.91631026 Link0_5_Drone_0 + Time_at_node_0_Drone_0
 - Time_at_node_5_Drone_0 = 985
_C94: - 1.32 Charge_at_node_0_Drone_0 + 1001.91631026 Link0_5_Drone_0
 + Time_at_node_0_Drone_0 - Time_at_node_5_Drone_0 = 860
_C95: 1000.857 Link1_2_Drone_0 + Time_at_node_1_Drone_0
 - Time_at_node_2_Drone_0 = 985
_C96: - 1.32 Charge_at_node_1_Drone_0 + 1000.857 Link1_2_Drone_0
 + Time_at_node_1_Drone_0 - Time_at_node_2_Drone_0 = 860
_C97: 1001.21198102 Link1_3_Drone_0 + Time_at_node_1_Drone_0
 - Time_at_node_3_Drone_0 = 985
_C98: - 1.32 Charge_at_node_1_Drone_0 + 1001.21198102 Link1_3_Drone_0
 + Time_at_node_1_Drone_0 - Time_at_node_3_Drone_0 = 860
_C99: 1000.857 Link1_4_Drone_0 + Time_at_node_1_Drone_0
 - Time_at_node_4_Drone_0 = 985
Bounds
-inf <= Charge_at_node_0_Drone_0 <= 100
-inf <= Charge_at_node_1_Drone_0 <= 100
-inf <= Charge_at_node_2_Drone_0 <= 100
-inf <= Charge_at_node_3_Drone_0 <= 100
-inf <= Charge_at_node_4_Drone_0 <= 100
-inf <= Charge_at_node_5_Drone_0 <= 100
-inf <= Charge_at_node_6_Drone_0 <= 100
-inf <= Charge_at_node_7_Drone_0 <= 100
0 <= Link0_1_Drone_0 <= 2
0 <= Link0_2_Drone_0 <= 2
0 <= Link0_3_Drone_0 <= 2
0 <= Link0_4_Drone_0 <= 2
0 <= Link0_5_Drone_0 <= 2
0 <= Link0_6_Drone_0 <= 2
0 <= Link0_7_Drone_0 <= 2
0 <= Link1_0_Drone_0 <= 2
0 <= Link1_2_Drone_0 <= 2
0 <= Link1_3_Drone_0 <= 2
0 <= Link1_4_Drone_0 <= 2
0 <= Link1_5_Drone_0 <= 2
0 <= Link1_6_Drone_0 <= 2
0 <= Link1_7_Drone_0 <= 2
0 <= Link2_0_Drone_0 <= 2
0 <= Link2_1_Drone_0 <= 2
0 <= Link2_3_Drone_0 <= 2
0 <= Link2_4_Drone_0 <= 2
0 <= Link2_5_Drone_0 <= 2
0 <= Link2_6_Drone_0 <= 2
0 <= Link2_7_Drone_0 <= 2
0 <= Link3_0_Drone_0 <= 2
0 <= Link3_1_Drone_0 <= 2
0 <= Link3_2_Drone_0 <= 2
0 <= Link3_4_Drone_0 <= 2
0 <= Link3_5_Drone_0 <= 2
0 <= Link3_6_Drone_0 <= 2
0 <= Link3_7_Drone_0 <= 2
0 <= Link4_0_Drone_0 <= 2
0 <= Link4_1_Drone_0 <= 2
0 <= Link4_2_Drone_0 <= 2
0 <= Link4_3_Drone_0 <= 2
0 <= Link4_5_Drone_0 <= 2
0 <= Link4_6_Drone_0 <= 2
0 <= Link4_7_Drone_0 <= 2
0 <= Link5_0_Drone_0 <= 2
0 <= Link5_1_Drone_0 <= 2
0 <= Link5_2_Drone_0 <= 2
0 <= Link5_3_Drone_0 <= 2
0 <= Link5_4_Drone_0 <= 2
0 <= Link5_6_Drone_0 <= 2
0 <= Link5_7_Drone_0 <= 2
0 <= Link6_0_Drone_0 <= 2
0 <= Link6_1_Drone_0 <= 2
0 <= Link6_2_Drone_0 <= 2
0 <= Link6_3_Drone_0 <= 2
0 <= Link6_4_Drone_0 <= 2
0 <= Link6_5_Drone_0 <= 2
0 <= Link6_6_Drone_0 <= 2
0 <= Link6_7_Drone_0 <= 2
0 <= Link7_0_Drone_0 <= 2
0 <= Link7_1_Drone_0 <= 2
0 <= Link7_2_Drone_0 <= 2
0 <= Link7_3_Drone_0 <= 2
0 <= Link7_4_Drone_0 <= 2
0 <= Link7_5_Drone_0 <= 2
0 <= Link7_6_Drone_0 <= 2
0 <= Link7_7_Drone_0 <= 2
Time_at_node_0_Drone_0 free
Time_at_node_1_Drone_0 free
Time_at_node_2_Drone_0 free
Time_at_node_3_Drone_0 free
Time_at_node_4_Drone_0 free
Time_at_node_5_Drone_0 free
Time_at_node_6_Drone_0 free
Time_at_node_7_Drone_0 free
Generals
Link0_1_Drone_0
Link0_2_Drone_0
Link0_3_Drone_0
Link0_4_Drone_0
Link0_5_Drone_0
Link0_6_Drone_0
Link0_7_Drone_0
Link1_0_Drone_0
Link1_2_Drone_0
Link1_3_Drone_0
Link1_4_Drone_0
Link1_5_Drone_0
Link1_6_Drone_0
Link1_7_Drone_0
Link2_0_Drone_0
Link2_1_Drone_0
Link2_3_Drone_0
Link2_4_Drone_0
Link2_5_Drone_0
Link2_6_Drone_0
Link2_7_Drone_0
Link3_0_Drone_0
Link3_1_Drone_0
Link3_2_Drone_0
Link3_4_Drone_0
Link3_5_Drone_0
Link3_6_Drone_0
Link3_7_Drone_0
Link4_0_Drone_0
Link4_1_Drone_0
Link4_2_Drone_0
Link4_3_Drone_0
Link4_5_Drone_0
Link4_6_Drone_0
Link4_7_Drone_0
Link5_0_Drone_0
Link5_1_Drone_0
Link5_2_Drone_0
Link5_3_Drone_0
Link5_4_Drone_0
Link5_6_Drone_0
Link5_7_Drone_0
Link6_0_Drone_0
Link6_1_Drone_0
Link6_2_Drone_0
Link6_3_Drone_0
Link6_4_Drone_0
Link6_5_Drone_0
Link6_6_Drone_0
Link6_7_Drone_0
Link7_0_Drone_0
Link7_1_Drone_0
Link7_2_Drone_0
Link7_3_Drone_0
Link7_4_Drone_0
Link7_5_Drone_0
Link7_6_Drone_0
Link7_7_Drone_0
End
