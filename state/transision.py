def decide_state(marker_list):
    if 10 in marker_list:
        # 左90°旋回用マーカー
        return "marker10"
    elif 11 in marker_list:
        # 右90°旋回用マーカー
        return "marker11"
    elif 0 in marker_list:
        # 充電へ移行するマーカー
        return "charge"
    else:
        # 空列
        return None