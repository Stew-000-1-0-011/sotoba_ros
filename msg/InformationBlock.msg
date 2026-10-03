# 同時ガウス事前/事後の情報行列のブロック1つ。
#
# i == j なら対角ブロック (そのオブジェクト自身の情報)、
# i != j ならオブジェクト間の相互相関ブロック。
#
# information は 6x6 の行優先 (情報行列 Λ のブロック)。
# 添字の順序は sotoba (Sophus::SE3f::Tangent) と同じ **[並進3, 回転3]**。
# ROS の PoseWithCovariance と並びは同じだが、あちらは共分散なので逆数的な意味を持つ。
# 摂動は左 (センサ座標系) で、対応する平均のまわりの局所座標 (T = exp(xi) mean)。

# (フィールド名を lambda にしないこと: Python の予約語で、生成される
#  Python バインディングが構文エラーになる)

uint8 i
uint8 j
float64[36] information
