def reward_function(params):
  var_1 = [*range(114,117,1),*range(0,17,1)]
  my_1u = [*range(18,43,1)]
  my_23 = [*range(44,73,1)]
  my_4 = [*range(74,90,1)]
  var_2 = [*range(91,100,1)]
  my_5 = [*range(101,113,1)]
  right = [*range(1,25,1),*range(50,57,1),*range(95,100,1)]
  center = [*range(28,32,1),*range(44,50,1),*range(59,62,1),*range(75,80,1),*range(91,95,1),*range(103,104,1),*range(115,117,1)]
  left = [*range(34,43,1),*range(65,72,1),*range(83,88,1),*range(105,110,1)]
  reward = 0
  p = params['closest_waypoints'][1]
  if params['all_wheels_on_track']:
    reward += 100
  else:
    reward += 0
  if p in left and params['is_left_of_center'] and params['distance_from_center'] >= params['track_width'] * 0.25 and params['distance_from_center'] <= params['track_width'] * 0.4:
    reward += 4
  elif p in right and not params['is_left_of_center'] and params['distance_from_center'] >= params['track_width'] * 0.25 and params['distance_from_center'] <= params['track_width'] * 0.4:
    reward += 5
  elif p in center and params['distance_from_center'] <= params['track_width'] * 0.1:
    reward += 5
  elif p in left and params['is_left_of_center'] and params['distance_from_center'] >= params['track_width'] * 0.1 and params['distance_from_center'] < params['track_width'] * 0.25:
    reward += 2
  elif p in right and not params['is_left_of_center'] and params['distance_from_center'] >= params['track_width'] * 0.1 and params['distance_from_center'] < params['track_width'] * 0.25:
    reward += 2
  elif p in center and params['distance_from_center'] >= params['track_width'] * 0.1 and params['distance_from_center'] < params['track_width'] * 0.25:
    reward += 2
  else:
    reward += 0
  if p in var_1 and params['speed'] <= 4 and params['speed'] >= 3.5:
    reward += 6
  elif p in my_1u and params['speed'] <= 3 and params['speed'] >= 2:
    reward += 5
  elif p in my_23 and params['speed'] <= 4 and params['speed'] >= 3.5:
    reward += 6
  elif p in my_4 and params['speed'] <= 3 and params['speed'] >= 2:
    reward += 5
  elif p in var_2 and params['speed'] <= 4 and params['speed'] >= 3.5:
    reward += 6
  elif p in my_5 and params['speed'] <= 3.5 and params['speed'] >= 2.5:
    reward += 5
  elif p in var_1 and params['speed'] < 3.5 and params['speed'] >= 2.5:
    reward += 4
  elif p in my_1u and params['speed'] < 2 and params['speed'] >= 1.5:
    reward += 3
  elif p in my_23 and params['speed'] < 3.5 and params['speed'] >= 2.5:
    reward += 4
  elif p in my_4 and params['speed'] < 2 and params['speed'] >= 1.5:
    reward += 3
  elif p in var_2 and params['speed'] < 3.5 and params['speed'] >= 2.5:
    reward += 4
  elif p in my_5 and params['speed'] < 2.5 and params['speed'] >= 1.5:
    reward += 3
  elif p in var_1 and params['speed'] < 2.5 and params['speed'] >= 1.5:
    reward += 1.5
  elif p in my_1u and params['speed'] < 1.5 and params['speed'] >= 1:
    reward += 1
  elif p in my_23 and params['speed'] < 2.5 and params['speed'] >= 1.5:
    reward += 1.5
  elif p in my_4 and params['speed'] < 1.5 and params['speed'] >= 1:
    reward += 1
  elif p in var_2 and params['speed'] < 2.5 and params['speed'] >= 1.5:
    reward += 1.5
  elif p in my_5 and params['speed'] < 1.5 and params['speed'] >= 1:
    reward += 1
  else:
    reward += 0
  if p in var_1 and params['steering_angle'] >= 0 and params['steering_angle'] <= 0:
    reward += 5
  elif p in my_1u and params['steering_angle'] >= 10 and params['steering_angle'] <= 20:
    reward += 6
  elif p in my_23 and params['steering_angle'] >= -5 and params['steering_angle'] <= 5:
    reward += 4
  elif p in my_4 and params['steering_angle'] >= 0 and params['steering_angle'] <= 20:
    reward += 5.5
  elif p in var_2 and params['steering_angle'] >= 0 and params['steering_angle'] <= 0:
    reward += 5
  elif p in my_5 and params['steering_angle'] >= 0 and params['steering_angle'] <= 20:
    reward += 5
  elif p in var_1 and params['steering_angle'] >= -3 and params['steering_angle'] <= 5:
    reward += 3
  elif p in my_23 and params['steering_angle'] >= -10 and params['steering_angle'] <= 10:
    reward += 3
  elif p in var_2 and params['steering_angle'] >= -3 and params['steering_angle'] <= 5:
    reward += 3
  else:
    reward += 0
  item += (params['progress'] / params['steps']) * 100
  return float(reward)