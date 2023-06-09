def get_obs(x_data,y_data):
    stable_segment = []
    increase_segment = []
    for i in range(len(x_data-1)):
        if abs(x_data[i] - x_data[i+1]) < 0.01:
            stable_segment.append(x_data[i],x_data[i+1])
        elif abs(x_data[i]-x_data[i+1]) > 0.05:
            increase_segment.append(x_data[i])
            
