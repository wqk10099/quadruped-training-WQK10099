#include <torch/script.h>

#include <iostream>
#include <memory>

int main(int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <model_path>\n";
    return 1;
  }

  torch::jit::script::Module module;

  try
  {
    module = torch::jit::load(argv[1]);
  }
  catch (const c10::Error & e)
  {
    std::cerr << "TorchScript load failed: " << e.what() << std::endl;
    return 1;
  }

  auto input = torch::zeros({1, 270});

  torch::NoGradGuard no_grad;
  auto output = module.forward({input}).toTensor();

  std::cout << "input shape:  " << input.sizes() << std::endl;
  std::cout << "output shape: " << output.sizes() << std::endl;
  std::cout << "output value:\n" << output << std::endl;

  return 0;
}