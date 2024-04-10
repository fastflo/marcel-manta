{'current_cell': 2, 'window_position': (860, 862), 'window_size': (856, 533)}## nb properties ##
{'current_cell': 2, 'window_position': (860, 862), 'window_size': (856, 533)}
## nb properties end ##
## cell 0 input ##
upper = 10e3
upper_v = 3.3
target_v = 1.4

lower_f = target_v / upper_v
upper_f = 1 - lower_f

% lower_f
% upper_f

lower = upper / upper_f * lower_f
% lower
## cell 0 output ##
# lower_f: 0.42424242424242425
# upper_f: 0.5757575757575757
# lower: 7368.421052631579
# 
## cell 0 end ##
## cell 1 input ##
% upper_v / (upper + lower) * lower
## cell 1 output ##
# upper_v / (upper + lower) * lower: 1.4
# 
## cell 1 end ##
## cell 2 input ##
#test = 4.7e3
test = 47e3
#test = 100e3
lower_test = 1 / (1/10e3 + 1/test)
% lower_test
% upper_v / (upper + lower_test) * lower_test
## cell 2 output ##
# lower_test: 8245.61403508772
# upper_v / (upper + lower_test) * lower_test: 1.4913461538461539
# 
## cell 2 end ##
