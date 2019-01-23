# Project 4: Follow Me

## Fully Convolution Network

This is my understanding of FCN:

 1. The encoder part transforms an image into the feature maps.
 2. The 1x1 convolution combines the feature maps and the decoder part.
 3. The decoder part upsamples the result back to the same size as the input image.

![img1][img1]

### Encoder Part

The encoder part will transform the input image to semantic representation. We can add multiple layers to make the network finding out generic representation.

![img2][img2]

The encode code looks like:

```python
def separable_conv2d_batchnorm(input_layer, filters, strides=1):
    output_layer = SeparableConv2DKeras(filters=filters,kernel_size=3, strides=strides, padding='same', activation='relu')(input_layer)
    output_layer = layers.BatchNormalization()(output_layer) 
    return output_layer
def encoder_block(input_layer, filters, strides):
    # Creates a separable convolution layer using the separable_conv2d_batchnorm() function.
    output_layer = separable_conv2d_batchnorm(input_layer, filters, strides=strides) 
    return output_layer
```

### 1x1 Convolution

The 1x1 convolution is between the encode and decode parts. Here is the code looks like:

```python
def conv2d_batchnorm(input_layer, filters, kernel_size=3, strides=1):
    output_layer = layers.Conv2D(filters=filters, kernel_size=kernel_size, strides=strides, padding='same', activation='relu')(input_layer)
        output_layer = layers.BatchNormalization()(output_layer) 
    return output_layer
```

### Decoder Part

The decoder part will upsample the 1x1 convolution to the same size as the input image. We will use the same layers to transpose the data. 

Also I will use skip connections to get more accurate segmentation decisions.

![img3][img3]

The decoder code looks like:

```python
def decoder_block(small_ip_layer, large_ip_layer, filters):
    # Upsample the small input layer using the bilinear_upsample() function.
    upsampled_layer = bilinear_upsample(small_ip_layer)
    # Concatenate the upsampled and large input layers
    output_layer = layers.concatenate([upsampled_layer, large_ip_layer])
    # Add some number of separable convolution layers
    output_layer = separable_conv2d_batchnorm(output_layer, filters)
    return output_layer
```

### Why and When

[From the Lesson 35/8] The encoder portion is a convolution network that reduces to a deeper 1x1 convolution layer. It reduces in the number of parameters. The reduction in the parameters make separable convolutions quite efficient with improved runtime performance, they also have the added benefit of reducing overfitting to an extent, because of the fewer parameters.

### Differences between 1x1 and fully connected layers

- First, 1x1 covolution works for different image size, but the fully connected layers only works for fixed size.
- The 1x1 covolution keeps spacial information, but the fully connected layers will transform all dimensions to a single vector, which will lose the spacial information.

## Model layer

First I need to figure out how many layers I need for the model size.

This is the situation for one layer:

![img4][img4]

And this is for 2 layers:

![img5][img5]

I modified the `fcn_model` function to add layers. The code should look like: 

```python
## Tis example has 3 layers because finally I pick 3 model layers.
def fcn_model(inputs, num_classes):
    
    filter = 26 #24 #32
    
    # TODO Add Encoder Blocks. 
    # Remember that with each encoder layer, the depth of your model (the number of filters) increases.
    encoder1 = encoder_block(inputs, filters=filter, strides=2)
    encoder2 = encoder_block(encoder1, filters=filter * 2, strides=2)
    encoder3 = encoder_block(encoder2, filters=filter * 4, strides=2)
    
    # TODO Add 1x1 Convolution layer using conv2d_batchnorm().
    conv1 = conv2d_batchnorm(encoder3, filters=filter * 4, kernel_size=1, strides=1)
    
    # TODO: Add the same number of Decoder Blocks as the number of Encoder Blocks
    decoder1 = decoder_block(small_ip_layer=conv1, large_ip_layer=encoder2, filters=filter * 4)
    decoder2 = decoder_block(small_ip_layer=decoder1, large_ip_layer=encoder1, filters=filter * 2)
    decoder3 = decoder_block(small_ip_layer=decoder2, large_ip_layer=inputs, filters=filter)
    
    # The function returns the output layer of your model. "x" is the final layer obtained from the last decoder_block()
    return layers.Conv2D(num_classes, kernel_size=1, activation='softmax', padding='same')(decoder3)
```

To start testing the model layer. I randomly initialized some parameters as:

```python
## randomly initalized parameters
learning rate = 0.1
batch size = 16
epochs = 5
validation steps = 100
```

I tried to modified the default values a little bit to make them look like closer to the first sense.

Here the results:

Layer No. | loss | val_loss | weight | IoU | score
-|-|-|-|-|-
1 | 0.06 | 0.082 | 0.7098445595854922 | 0.302873399539 | 0.214993034906
2 | 0.0472 | 0.0586 | 0.6825396825396826 | 0.334882189654 | 0.228570383415
3 | 0.041 | 0.053 | 0.6547368421052632 | 0.450711088192 | 0.295097154584
4 | 0.0393 | 0.1481 | 0.6541095890410958 | 0.406761774809 | 0.266066777358

**Table 1: Pick the layer number** 

Looks like when `layer = 3` I got the best result. So I decided to pick model layer 3.

## Parameters

The are 5 parameters but I only need to figure out 3 of them. `steps_per_epoch` is recommended to use total image number divided by the batch_size, so it always be `4131/batch_size`. (4131 is the image number in my `./data/train/images` folder). And the `workers` is recommended to use 2.

## Batch Size

I assume the batch size controls the noise of the image in the middle layers. So ideal the higher the better. But the higher the batch size, the more time I need to compute it, and the time consumed is not worth for the noise improvement. I decided to try 16, 32 and 64. 

## Learning Rate

I think the learning rate is not linear. Start from `0.1`, I decided to try the rate like `0.1, 0.05, 0.01, 0.005...`. After I figure out the "best" rate I think, I will try a little more around that value. For instance, I found 0.01 matched my expectation, so I also tried 0.02 and 0.008.

## Number of Epochs

I think the epoch number just simple increase the compute time to make a better result. Each computation will cost me about 30 to 45 seconds, so during the test I use 5 for most of the time. After I have decided other parameters. I will increase the epoch number linearly to try to figure out the best value but less computation.

`Review: You should add more information to your results section, it should contain a discussion on the different results achieved (different metrics), and what can be done to improve it.`

First, I tried to find out the batch size value. Actully I am not quite sure how to find out the best value because I don't know how this parameter will impact the final socre. Firstly I think using 1 is good enought. I did some research online then I noticed someone suggesting using sample number divided around 200 (or 150-250) to get better practice. So I start from 16. I also tried 32 and 64 but I didn't see any obvious different in final socre. So I randomly using 16 and 32 for the rest of the computation.

Then I started to find out the learning rate. I use the default values for other parameters, then tried the following rate: `0.1, 0.05, 0.01, 0.005, 0.02, 0.011, 0.009, 0.008`. I noticed that when I pick 0.01, the final score did not change much but I can get the best `loss`. Also when I pick `0.009` or `0.011`, I cannot see much different (which is not recorded in the table below). So I dedice to chose `0.01`.

Secondarily, I try to find out the best epochs number. As I discribed above, I think the epochs just increase the compute time to get better `loss`. I tried `5, 10, 15, 20, 30, 40, 50` (some are not recorded in table). I noticed that after `epoch > 30`, the `loss` does not change much. From 5 to 30, the average `loss` reduced around from `0.065` to `0.035`, but from `30` to `50`, it only reduced from `0.035` to `0.033`. So I thing the best choice should between 30 and 50. I randomly using 30 and 50 for the rest of my work. I think it will not impact the result very much. But I didn't try any value larger than 50 because I think it must be overfitting.

learning rate | batch size | epochs | validation steps | loss | val_loss | weight | IoU | score
-|-|-|-|-|-|-|-|-|-|-|-
0.1 | 16 | 5 | 50 | 0.0444 | 0.0531 | 0.6421370967741935 | 0.454386835853 | 0.291778643587
0.05 | 16 | 5 | 50 | 0.0434 | 0.0958 | 0.6820234869015357 | 0.469013955786 | 0.319878533531
0.01 | 16 | 5 | 50 | 0.0357 | 0.0332 | 0.6658878504672897 | 0.367571894119 | 0.244761658467
0.005 | 16 | 5 | 50 | 0.0575 | 0.0661 | 0.6510359869138496 | 0.368213322592 | 0.239720123869
0.02 | 16 | 5 | 50 | 0.037 | 0.0712 | 0.46730083234244946 | 0.158150811559 | 0.0739040058771
0.05 | 16 | 5 | 100 | 0.0455 | 0.1451 | 0.7137745974955277 | 0.437803765355 | 0.312493206399
0.01 | 16 | 5 | 100 | 0.0354 | 0.045 | 0.6829545454545455 | 0.431924422003 | 0.2949847473
0.01 | 16 | 20 | 100 | 0.0226 | 0.0319 | 0.7043756670224119 | 0.517325120235 | 0.364391226633
0.05 | 16 | 30 | 100 | 0.0215 | 0.036 | 0.7337962962962963 | 0.501672804747 | 0.368125646076
0.05 | 16 | 50 | 100 | 0.0194 | 0.0369 | 0.6776765375854215 | 0.368928974284 | 0.250014509908
0.01 | 16 | 50 | 100 | 0.0191 | 0.0328 | 0.7100456621004566 | 0.486063625738 | 0.34512736896
0.01 | 16 | 30 | 100 | 0.0206 | 0.0379 | 0.7079463364293086 | 0.566117912902 | 0.400781102426
0.02 | 16 | 30 | 100 | 0.0208 | 0.0335 | 0.7144444444444444 | 0.510985384005 | 0.365070668795
0.01 | 32 | 30 | 100 | 0.0201 | 0.03 | 0.7374429223744292 | 0.538162499853 | 0.396864126604
0.01 | 32 | 50 | 100 | 0.0168 | 0.0303 | 0.7543290043290043 | 0.577682242096 | 0.435762470499
0.01 | 64 | 50 | 100 | 0.0159 | 0.0287 | 0.7226027397260274 | 0.52320324019 | 0.378068094794
0.01 | 32 | 50 | 50 | 0.0182 | 0.0253 | 0.7190426638917794 | 0.579423368408 | 0.416630122341
0.01 | 32 | 50 | 50 | 0.0174 | 0.0343 | 0.7307692307692307 | 0.589672779253 | 0.430914723301

**Table 2: Pick Parameters**

In fact, I randomly tested almost two times more than what I listed in the table. But most of them didn't have good score so I just ignore them. The last line in the table is the best result of all my test.

You will notice that there is a line which has higher socre than the last one. But I cannot reproduce that result any more. I am not sure why, maybe some cache problem or I recorded wrong number. Also I changed the decoder code a little bit during the test, which might cause that I cannot get score higher than 0.4 by using these parameters. But for the latest set, it always works find. So my final set is:

```python
learning_rate = 0.01
batch_size = 32 #16
num_epochs = 50 #50
steps_per_epoch = 4131/batch_size #4131 = total image number
validation_steps = 50 #100
workers = 2
```

## Results

The limitation of this model and data is that it's only work for the given samples because it's trained with the certain object. To create a model to solve  this limitation, I think the dataset needs to have enough examples for different classes (people and other objects). Anyhow, I have no idea how to implement it.

Another problem is that to figure out the parameters costed me lots of time. I have to run it and write done the results and compare them and then try to pick the better values. I was considering to write some code to run this automatically so that it can figure out what the best parameter set is but no idea how to start, because I noticed that compute all the possibilities is a huge computation cost, I don't think I can figure out the best method.

[img1]: ./imgs/img1.png
[img2]: ./imgs/img2.png
[img3]: ./imgs/img3.png
[img4]: ./imgs/img4.png
[img5]: ./imgs/img5.png
