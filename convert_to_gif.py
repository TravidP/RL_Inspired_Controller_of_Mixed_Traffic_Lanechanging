import os
from moviepy.editor import VideoFileClip

def convert_videos_to_gifs(video_dir, fps=10, resize_factor=0.5):
    """
    将目标文件夹下的所有 .mp4 和 .avi 视频转换为体积更小的 .gif 文件。
    
    参数:
        video_dir (str): 包含视频文件的文件夹路径。
        fps (int): GIF 的帧率，默认 10 可以在保证流畅度的前提下缩小体积。
        resize_factor (float): 画面缩放比例，默认 0.5 将长宽各缩小一半。
    """
    if not os.path.exists(video_dir):
        print(f"错误: 找不到文件夹 {video_dir}")
        return

    for filename in os.listdir(video_dir):
        if filename.endswith(('.mp4', '.avi')):
            video_path = os.path.join(video_dir, filename)
            gif_filename = os.path.splitext(filename)[0] + '.gif'
            gif_path = os.path.join(video_dir, gif_filename)

            print(f"正在转换 {filename} 为 GIF...")
            try:
                clip = VideoFileClip(video_path)
                clip = clip.resize(resize_factor) # 缩小分辨率以减小体积
                clip.write_gif(gif_path, fps=fps)
                print(f"✅ 成功保存: {gif_filename}\n")
            except Exception as e:
                print(f"❌ 转换 {filename} 失败: {e}\n")

if __name__ == "__main__":
    # 自动定位到当前脚本所在目录下的 Docs 文件夹
    docs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Docs')
    convert_videos_to_gifs(docs_dir, fps=10, resize_factor=0.5)