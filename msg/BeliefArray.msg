# オブジェクト群の信念分布 (同時ガウス)。
#
# header.stamp は「この平均と情報行列が指す時刻」。
# 事前として送る場合、受け手 (sotoba_node) はスキャン時刻までの差分を
# 自分の持続予測で埋めてから使う。したがって送り手はスキャン周期を知らなくてよい。
#
# names は順序の取り違えを防ぐためのもので、means / blocks の添字と対応する。
# blocks が空なら「情報ゼロ = 何も分からない」を意味する。

std_msgs/Header header
string[] names
geometry_msgs/Pose[] means
InformationBlock[] blocks

# 送り手だけが知っているヒント。空なら不明。
# 0=not_run, 1=updated, 2=too_few_correspondences, 3=solve_failed
uint8[] status
