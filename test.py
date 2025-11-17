import torch
if torch.cuda.is_available():
      print(f"Number of GPUs detected: {torch.cuda.device_count()}")
else:
      print("CUDA is not available, no GPUs detected by PyTorch.")