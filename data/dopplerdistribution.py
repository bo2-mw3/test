import rosbag
import numpy as np
import matplotlib.pyplot as plt

# bagファイルを開く
bag_path = '/home/kaneko.tatsushi/catkin_ws/src/hdl_localization/data/ds5deg.bag'
try:
    bag = rosbag.Bag(bag_path)
    print(f"bagファイル '{bag_path}' を正常に開きました。")

    # bagファイルの基本情報を表示
    info = bag.get_type_and_topic_info()
    print("\nbagファイルの情報:")
    print(f"  期間: {bag.get_end_time() - bag.get_start_time():.2f} 秒")
    print(f"  メッセージ数: {bag.get_message_count()}")
    print("\nトピック一覧:")
    for topic, topic_info in info.topics.items():
        print(f"  - {topic}: {topic_info.message_count} メッセージ")

    # '/EditRadar' トピックの存在確認
    if 'EditRadar' in info.topics:
        print("\n'EditRadar' トピックが見つかりました。")
        print(f"  メッセージ数: {info.topics['EditRadar'].message_count}")
        print(f"  タイプ: {info.topics['EditRadar'].msg_type}")
    else:
        print("\n警告: 'EditRadar' トピックが見つかりません。")



except Exception as e:
    print(f"エラー: bagファイルを開けませんでした。\n{str(e)}")
    exit()

# 対象とするフレーム番号
target_seq = 3730
print(f"\n対象フレーム番号: {target_seq}")

# EditRadarトピックのメッセージを確認
for topic, msg, t in bag.read_messages(topics=['EditRadar']):
    print("メッセージ構造:")
    print(f"  ヘッダー: {msg.header}")
    print(f"  ポイント数: {len(msg.points)}")
    print("  チャンネル:")
    for channel in msg.channels:
        print(f"    - 名前: {channel.name}, 値の数: {len(channel.values)}")
    
    # 1つのメッセージを確認したら終了
    break

bag.close()


# 対象とするフレーム番号
target_seq = 3730
# bagファイルを開く
bag = rosbag.Bag('/home/kaneko.tatsushi/catkin_ws/src/hdl_localization/data/ds5deg.bag')
# データを格納するリスト
points_data = []
doppler_data = []
# トピック名が'/EditRadar'で、その中に'points'、'Doppler'、および'header'チャンネルがあると仮定します
for topic, msg, t in bag.read_messages(topics=['EditRadar']):
    if msg.header.seq == target_seq:  # フレーム番号が2622の場合のみデータを取得
        # 'points'チャンネルから座標データを取得
        for point in msg.points:
            points_data.append([point.x, point.y, point.z])
        # 'Doppler'チャンネルからドップラー速度を取得
        #doppler_data.append(msg.channels.Doppler)
            for channel in msg.channels:
              if channel.name == 'Doppler':
                doppler_data.append(channel.values)
                break
bag.close()
# numpy配列に変換
points_data = np.array(points_data)
print(points_data.shape)
print(points_data)
doppler_data = np.array(doppler_data)
# データの形状を確認
print(f"points_data の形状: {points_data.shape}")
print(f"doppler_data の形状: {doppler_data.shape}")
# x, y座標
x = points_data[:, 0]
y = points_data[:, 1]
# ドップラー速度の大きさ
v = np.abs(doppler_data)
# x, y座標から角度θを計算（atan2は角度を-πからπの範囲で返す）
theta = np.arctan2(x, y)
# ドップラー速度のx, y方向の成分
vx = v * np.sin(theta)
vy = v * np.cos(theta)
# 図を描画
plt.scatter(vx, vy, color='blue',s=1)  # 点の位置をプロット
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Doppler Velocity Distribution (Frame 3730)')
plt.grid(True)
plt.show()
