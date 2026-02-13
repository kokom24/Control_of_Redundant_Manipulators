# Control_of_Redundant_Manipulators
粘弾性体マニピュレーターを用いた冗長マニピュレータの制御手法についてのシミュレーションコードです。\
これまでは、マニピュレータの各リンクを剛体としていた部分をバネとして見立てたモデルです。

## 【ファイル構成】
- ###  <Centralized_control：集中制御のシミュレータ>
    - arm_animetion:アニメーションを再生できるプログラム
    - joint_break：故障関節があったときの故障適応性を評価するプログラム
    - parameter_tuning：パラメータチューニングを行えるプログラム
- ### <Distributed_Control：分散知能のシミュレータ>
    - dis_animetion：アニメーションを再生できるプログラム
    - dis_joint_break：故障関節があったときの故障適応性を評価するプログラム
- ### ＜other＞
    - 上記以外で齊藤が作成・使いまわしていたコード達が集まっています。開発途中のversionもあるため、必要に応じて見る形にしてください。（本人も、どのversionが何ができていて何ができていないか覚えていないです...）\
\
以下概要スライドです。\

<img width="900" height="675" alt="image" src="https://github.com/user-attachments/assets/a50357e4-2d90-451f-9278-34cd72f3e775" />
<img width="900" height="675" alt="image" src="https://github.com/user-attachments/assets/4fd067dc-4b67-42c7-80ca-2555be988306" />
<img width="900" height="675" alt="image" src="https://github.com/user-attachments/assets/4dae3640-95b1-42fd-b865-ba71c5523fbd" />
<img width="900" height="675" alt="image" src="https://github.com/user-attachments/assets/7630f9c4-d90f-4c45-8fe9-29dfa9a182de" />
<img width="900" height="675" alt="image" src="https://github.com/user-attachments/assets/96300bb6-6908-4675-9bb1-7f9ea99bf435" />

